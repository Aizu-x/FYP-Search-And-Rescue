"""Fill these wrappers when the team's modules arrive; edit real_bindings.json.

Remove @not_connected only after implementing and testing each wrapper.
Signatures are always function(inputs, context). Return a JSON-compatible dict.
Inputs contains ONLY this task's successful direct prerequisites.
See INTEGRATION.md and contracts.py for the exact expected outputs.
"""


def not_connected(function):
    function.__dag_unconfigured__ = True
    return function


def prepare_recording(inputs, context):
    """Validate one of the supplied recorded Tello sequences; no live capture."""
    from orb_adapter import prepare_recording as prepare
    return prepare(inputs, context)


def run_slam(inputs, context):
    """Import verified saved exports or execute the compiled Linux runner."""
    from orb_adapter import run_slam as execute
    return execute(inputs, context)


@not_connected
def run_erfnet(inputs, context):
    """Call ERFNet for the keyframes listed by inputs['run_slam']."""
    raise NotImplementedError("Connect three-class ERFNet")


@not_connected
def fuse_map(inputs, context):
    """Combine SLAM geometry and ERFNet labels into the ground map."""
    raise NotImplementedError("Connect map fusion")


@not_connected
def prepare_planning_map(inputs, context):
    """Preserve forbidden areas and account for the MentorPi footprint."""
    raise NotImplementedError("Connect planning-map preparation")


@not_connected
def select_target(inputs, context):
    """Choose the mission's inspection target in the common arena frame."""
    raise NotImplementedError("Connect target selection")


@not_connected
def get_ugv_pose(inputs, context):
    """Obtain a fresh measured MentorPi position in the same map frame."""
    raise NotImplementedError("Connect robot localization")


def plan_path(inputs, context):
    """Call the team's custom A*. adapters.planning_output formats its result."""
    from ugv_adapter import plan_path as plan
    return plan(inputs, context)


@not_connected
def navigate(inputs, context):
    """Send route + request_id, wait for arrival/failure, observe obstacles.

    Use bounded network timeouts. Report only confirmed arrival. Raise
    ReplanRequested for an obstacle that requires a new planning attempt.
    The controller must support an independent stop request for this request_id.
    """
    raise NotImplementedError("Connect the robot movement controller")


@not_connected
def stop_navigation(inputs, context):
    """Stop the original request and return {stopped: True, request_id: ...}.

    Send context.request_id even if the worker died before sending its route.
    The robot controller must make this operation idempotent and remember
    cancellation so a late command for that request cannot restart movement.
    A lost connection is NOT a stop acknowledgement.
    """
    raise NotImplementedError("Connect confirmed robot stopping")


def inspect_target(inputs, context):
    """Run the supplied YOLOv5n on request-linked UGV images after arrival."""
    from ugv_adapter import inspect_target as inspect
    return inspect(inputs, context)
