"""실제 시나리오 함수만 추출해 실패 경로를 시험한다. ROS 명령·파일 기록은 하지 않는다."""

import ast
import io
from pathlib import Path
from types import SimpleNamespace

import pytest


SOURCE = Path(__file__).resolve().parents[3] / "mingi_control/transport_scenario.py"


def scenario_functions():
    tree = ast.parse(SOURCE.read_text())
    functions = [
        item
        for item in tree.body
        if isinstance(item, ast.FunctionDef)
        and item.name in ("call", "run_base_move", "abort_transport")
    ]
    scope = {}
    exec(
        compile(ast.Module(body=functions, type_ignores=[]), str(SOURCE), "exec"), scope
    )
    return scope, tree


def harness(event="DONE", reset_ok=True, start_ok=True):
    scope, tree = scenario_functions()
    calls, lifts = [], []
    clock = SimpleNamespace(value=0.0)
    base = {"v": True, "status": '{"state": "FAULT", "reason": "previous cycle"}'}

    def call(client, label):
        calls.append(label)
        return {"reset": reset_ok, "start": start_ok, "stop": True}[client]

    def spin(seconds):
        clock.value += seconds
        if "start_base_move" in calls:
            if event == "DONE":
                base.update(v=True, status='{"state": "DONE"}')
            elif event == "FAULT":
                base.update(
                    v=False, status='{"state": "FAULT", "reason": "feedback lost"}'
                )
            elif event == "BAD_STATUS":
                base.update(v=False, status="invalid status")

    scope.update(
        call=call,
        spin=spin,
        base_done=base,
        lift_pub=SimpleNamespace(publish=lambda msg: lifts.append(msg.data)),
        Bool=lambda **kwargs: SimpleNamespace(**kwargs),
        reset_cli="reset",
        start_cli="start",
        stop_cli="stop",
        events=[],
        a=SimpleNamespace(base_timeout=1.0),
        time=SimpleNamespace(
            monotonic=lambda: clock.value,
            time=lambda: clock.value,
            strftime=lambda fmt: "test",
        ),
        json=__import__("json"),
        stop=SimpleNamespace(set=lambda: calls.append("stop_monitor")),
        th=SimpleNamespace(join=lambda timeout: calls.append("join_monitor")),
        open=lambda *args: io.StringIO(),
        ev_log="not-written.csv",
        n=SimpleNamespace(destroy_node=lambda: calls.append("destroy_node")),
        rclpy=SimpleNamespace(try_shutdown=lambda: calls.append("shutdown")),
    )
    return scope, tree, calls, lifts


def run_guard(scope, tree):
    # 실제 최상위 try/except를 그대로 실행하고, 이후 단계 도달 여부를 검사한다.
    guard = next(
        item
        for item in tree.body
        if isinstance(item, ast.Try)
        and any(
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "run_base_move"
            for node in ast.walk(item)
        )
    )
    after = ast.parse("lowering_reached = True").body
    exec(
        compile(ast.Module(body=[guard, *after], type_ignores=[]), str(SOURCE), "exec"),
        scope,
    )


def test_completed_move_allows_lowering_and_clears_previous_cycle():
    scope, tree, calls, lifts = harness()
    run_guard(scope, tree)
    assert scope["lowering_reached"]
    assert calls == ["reset_base_move", "start_base_move"]
    assert lifts == [False, True]


@pytest.mark.parametrize(
    "event,reset_ok,start_ok",
    [
        ("FAULT", True, True),
        ("TIMEOUT", True, True),
        ("BAD_STATUS", True, True),
        ("DONE", False, True),
        ("DONE", True, False),
    ],
)
def test_failures_request_stop_and_never_reach_lowering(event, reset_ok, start_ok):
    scope, tree, calls, lifts = harness(event, reset_ok, start_ok)
    with pytest.raises(SystemExit) as error:
        run_guard(scope, tree)
    assert error.value.code == 1
    assert "lowering_reached" not in scope
    assert "stop_base_move" in calls
    assert lifts[-1] is False
    assert calls[-1] == "shutdown"


def test_stop_service_loss_still_prevents_lowering_and_cleans_up():
    scope, tree, calls, lifts = harness("FAULT")
    previous_call = scope["call"]

    def call(client, label):
        if client == "stop":
            raise TimeoutError("stop service unavailable")
        return previous_call(client, label)

    scope["call"] = call
    with pytest.raises(SystemExit):
        run_guard(scope, tree)
    assert "lowering_reached" not in scope
    assert lifts[-1] is False
    assert calls[-1] == "shutdown"


@pytest.mark.parametrize("failure", ["lift_publish", "event_log"])
def test_cleanup_errors_still_request_stop_and_prevent_lowering(failure):
    scope, tree, calls, _ = harness("FAULT")
    if failure == "lift_publish":
        publisher = scope["lift_pub"].publish

        def publish(message):
            if "start_base_move" in calls and not message.data:
                raise RuntimeError("publisher unavailable")
            publisher(message)

        scope["lift_pub"].publish = publish
    else:

        def failed_open(*args):
            raise OSError("log unavailable")

        scope["open"] = failed_open
    with pytest.raises(SystemExit) as error:
        run_guard(scope, tree)
    assert error.value.code == 1
    assert "lowering_reached" not in scope
    assert "stop_base_move" in calls
    assert calls[-1] == "shutdown"


def test_service_timeout_is_not_treated_as_success():
    scope, _ = scenario_functions()
    cancelled = []
    future = SimpleNamespace(done=lambda: False, cancel=lambda: cancelled.append(True))
    scope.update(
        Trigger=SimpleNamespace(Request=lambda: None),
        n=object(),
        rclpy=SimpleNamespace(spin_until_future_complete=lambda *args, **kwargs: None),
    )
    client = SimpleNamespace(
        wait_for_service=lambda timeout_sec: True, call_async=lambda req: future
    )
    with pytest.raises(TimeoutError):
        scope["call"](client, "start_base_move")
    assert cancelled == [True]
