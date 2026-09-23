"""No ROS dependencies: ownership-preserving integration with the existing mission."""
def attach_advisory(node, observe):
    alignment = node.passage_alignment
    original = alignment.perform
    def perform(candidate, deadline):
        if node.active_goal_handle is not None:
            raise RuntimeError('Vision handoff requires settled navigation')
        node.recovery_guard.ready()
        observe(candidate, deadline)
        node.recovery_guard.ready()
        return original(candidate, deadline)
    alignment.perform = perform


def execute_leg(node, mode, recover_first=False):
    if recover_first:
        if not node.recovery_guard.backup('operator-requested reposition before HOME'):
            raise RuntimeError('Existing audited backup refused or failed; HOME not dispatched')
    target = (0., 0., 0.) if mode == 'return-home' else (28., 82., 1.3028507292104001)
    name = 'HOME' if mode == 'return-home' else 'DELIVERY_C'
    # Use the existing navigation entry point, including alignment and recovery.
    return node.navigate_with_exploration(name, target)
