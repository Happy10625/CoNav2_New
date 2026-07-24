from types import SimpleNamespace

import math

from co_nav2_nav.semantic_explorer import SemanticExplorer, normalize_angle


class _Logger:
    def warn(self, _message):
        pass

    def info(self, _message):
        pass


class _PendingFuture:
    def __init__(self):
        self.callback = None

    def add_done_callback(self, callback):
        self.callback = callback


class _ActionClient:
    def __init__(self):
        self.goals = []

    def wait_for_server(self, timeout_sec):
        assert timeout_sec == 0.5
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return _PendingFuture()


def test_normalize_angle_chooses_short_rotation_across_pi_boundary():
    assert math.isclose(normalize_angle(math.radians(370)), math.radians(10))
    assert math.isclose(normalize_angle(math.radians(-370)), math.radians(-10))


def test_scan_uses_relative_spin_goal_with_bounded_time_allowance():
    node = SemanticExplorer.__new__(SemanticExplorer)
    node.spinner = _ActionClient()
    node.p = SimpleNamespace(spin_time_allowance=20.0)
    node.goal_token = 0
    node.get_logger = lambda: _Logger()

    node.send_spin(math.pi / 4.0, "scan")

    assert len(node.spinner.goals) == 1
    goal = node.spinner.goals[0]
    assert math.isclose(goal.target_yaw, math.pi / 4.0)
    assert goal.time_allowance.sec == 20
    assert goal.time_allowance.nanosec == 0
    assert node.goal_kind == "scan"
    assert node.goal_pending is True
    assert node.goal_pose is None


def test_approach_plan_still_dispatches_navigate_to_pose():
    node = SemanticExplorer.__new__(SemanticExplorer)
    node.plan_token = 4
    node.current_plan_pose = (1.0, 0.0, 0.0)
    node.session_origin = (0.0, 0.0)
    node.p = SimpleNamespace(max_travel_radius=3.0)
    node.plan_kind = "approach"
    node.plan_pending = True
    node.plan_handle = object()
    node.plan_candidates = [(2.0, 0.0, 0.0)]
    states = []
    navigation_goals = []
    node.set_state = states.append
    node.send_navigation_goal = lambda *goal: navigation_goals.append(goal)
    path = [
        SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0))),
        SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.0, y=0.0))),
    ]
    future = SimpleNamespace(
        result=lambda: SimpleNamespace(
            status=4, result=SimpleNamespace(path=SimpleNamespace(poses=path))))

    node.on_plan_result(future, 4)

    assert states == ["APPROACHING"]
    assert navigation_goals == [(1.0, 0.0, 0.0, "approach")]
    assert node.plan_pending is False


def test_exhausted_approach_plans_stop_in_failed_state():
    node = SemanticExplorer.__new__(SemanticExplorer)
    node.plan_token = 7
    node.plan_candidates = []
    node.plan_kind = "approach"
    node.plan_pending = True
    node.plan_handle = object()
    node.get_logger = lambda: _Logger()
    failures = []
    node.fail_safe = failures.append

    node._request_next_plan(7)

    assert failures == ["Nav2 cannot plan a path within the target clearance"]
    assert node.plan_pending is False
    assert node.plan_handle is None
    assert node.plan_kind is None


def test_failed_approach_goal_uses_fail_safe_instead_of_resuming_search():
    node = SemanticExplorer.__new__(SemanticExplorer)
    node.goal_token = 3
    node.goal_pose = (1.0, 2.0, 0.0)
    node.goal_handle = object()
    node.goal_pending = True
    node.goal_kind = "approach"
    node.blocked_goals = {}
    node.get_logger = lambda: _Logger()
    node.goal_key = SemanticExplorer.goal_key
    failures = []
    node.fail_safe = failures.append
    future = SimpleNamespace(
        result=lambda: SimpleNamespace(status=6))

    node.on_goal_result(future, 3, "approach")

    assert failures == ["Approach goal failed with status 6"]
