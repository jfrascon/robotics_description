"""
Reusable helpers for launch files across eut_robotics_description.

These functions return Launch Substitutions or perform runtime validation
so that launch scripts can avoid duplicating logic.
"""

from __future__ import annotations

from launch import LaunchContext, LaunchDescriptionEntity
from launch.actions import SetLaunchConfiguration
from launch.substitution import Substitution
from launch.substitutions import (
    EqualsSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
    OrSubstitution,
    PythonExpression,
    TextSubstitution,
)

# ------------------------------------------------------------------------------
# Regular functions
# ------------------------------------------------------------------------------


def validate_namespace(ns: str) -> None:
    """
    Validate a namespace string.

    Rules:
    - Reject ASCII spaces.
    - Reject empty segments (no '//' allowed).
    - Each segment must be ASCII alnum or underscore.
    """

    if ns in ('', '/'):
        return

    if ' ' in ns:
        raise ValueError("Namespace cannot contain the ASCII space character ' '")

    # In order to check if there are two or more '/' in a row, we need to first remove the leading and trailing slashes
    # if present.
    # Remove exactly one leading slash.
    if ns.startswith('/'):
        ns = ns[1:]

    # Remove exactly one trailing slash
    if ns.endswith('/'):
        ns = ns[:-1]

    parts = ns.split('/') if ns else []

    for seg in parts:
        if seg == '':
            raise ValueError("Consecutive '/' are not allowed in a namespace")
        if not all((c == '_') or (c.isascii() and c.isalnum()) for c in seg):
            raise ValueError('Namespace segments must be ASCII [A-Za-z0-9_] only')


# ------------------------------------------------------------------------------
# Substitutions.
# ------------------------------------------------------------------------------


def make_robot_namespace(namespace_subst: Substitution | str, robot_name_subst: Substitution | str) -> Substitution:
    """
    Build a fully-qualified robot namespace as a Substitution.

    It safely joins a (possibly root) namespace with the robot name, avoiding double slashes by using rstrip on the
    namespace.

    Examples
    - Create and log:
        rn = make_robot_namespace(LaunchConfiguration('namespace'), LaunchConfiguration('robot_name'))
        LogInfo(msg=['Robot namespace: ', rn])

    - Use in include/node args: ::
        'topic': [rn, '/robot_description']
    """
    # The 'robot_namespace' is the concatenation of the 'namespace' and the 'robot_name', using the character '/' a
    # separtor.
    # namespace=''          -> robot_namespace = robot_name
    # namespace='/'         -> robot_namespace = '/' + robot_name
    # namespace='ns'        -> robot_namespace = 'ns' + '/' + robot_name
    # namespace='ns/'       -> robot_namespace = 'ns' + '/' + robot_name
    # namespace='/ns/'      -> robot_namespace = '/ns' + '/' + robot_name
    # namespace='/ns1/ns2'  -> robot_namespace = '/ns1/ns2' + '/' + robot_name
    # namespace='/ns1/ns2/' -> robot_namespace = '/ns1/ns2' + '/' + robot_name

    is_empty = EqualsSubstitution(namespace_subst, '')

    return IfElseSubstitution(
        condition=is_empty,
        if_value=[robot_name_subst],
        else_value=PythonExpression(["'", namespace_subst, "'.rstrip('/') + '/", robot_name_subst, "'"]),
    )


def make_robot_prefix(namespace_subst: Substitution | str, robot_name_subst: Substitution | str) -> Substitution:
    """
    Build the robot prefix Substitution, flattening namespace with '_'.

    - If namespace is '' or '/', prefix becomes '<robot_name>_'
    - Else: '<ns_flat>_<robot_name>_'
      where ns_flat = namespace.strip('/').replace('/', '_')

    Example
    - rp = make_robot_prefix(LaunchConfiguration('namespace'), LaunchConfiguration('robot_name'))

    Use rp to parameterize joints/links.
    """
    # The 'prefix' is similar to the 'namespace', it is a 'flatenized' version of the namespace, i.e., it uses the
    # character '_' as a separator instead of the character '/'.
    # The 'robot_prefix' is the concatenation of the 'prefix' and the 'robot_name', # using the character '_' as a
    # separator.

    # namespace=''          -> robot_prefix = robot_name + '_'
    # namespace='/'         -> robot_prefix = robot_name + '_'
    # namespace='ns'        -> robot_prefix = 'ns' + '_' + robot_name + '_'
    # namespace='ns/'       -> robot_prefix = 'ns' + '_' + robot_name + '_'
    # namespace='/ns/'      -> robot_prefix = 'ns' + '_' + robot_name + '_'
    # namespace='/ns1/ns2'  -> robot_prefix = 'ns1_ns2' + '_' + robot_name + '_'
    # namespace='/ns1/ns2/' -> robot_prefix = 'ns1_ns2' + '_' + robot_name + '_'
    ns_is_empty_or_slash = OrSubstitution(
        EqualsSubstitution(namespace_subst, ''), EqualsSubstitution(namespace_subst, '/')
    )

    return IfElseSubstitution(
        condition=ns_is_empty_or_slash,
        if_value=[robot_name_subst, TextSubstitution(text='_')],
        else_value=PythonExpression(
            ["'", namespace_subst, "'.strip('/').replace('/', '_') + '_' + '", robot_name_subst, "_'"]
        ),
    )


# ------------------------------------------------------------------------------
# Opaque functions.
# ------------------------------------------------------------------------------


def to_global_namespace(ctx: LaunchContext, ns_key: str = 'namespace') -> list[LaunchDescriptionEntity]:
    """
    Normalize and validate LaunchConfiguration(ns_key) to a global form.

    Semantics:
    - ''  -> '/'
    - '/' -> '/'
    - 'ns', '/ns', 'ns/', '/ns/' -> '/ns'
    - Reject spaces, consecutive '/', and non [A-Za-z0-9_] chars in segments.

    Returns a list with a SetLaunchConfiguration when it must update the value;
    otherwise returns an empty list if no change is required.
    """
    ns = LaunchConfiguration(ns_key).perform(ctx)

    # Empty becomes root '/'
    if ns == '':
        return [SetLaunchConfiguration(ns_key, '/')]

    # Already root
    if ns == '/':
        return []

    # A namespace should not end in a '/'; it is not an error if the namespace ends with a '/', but it is convenient to
    # to remove if it is present '/'.
    ns_norm = ns[:-1] if ns.endswith('/') else ns

    # A global namespace must start with a '/'.
    if not ns_norm.startswith('/'):
        ns_norm = '/' + ns_norm

    validate_namespace(ns_norm)

    return [SetLaunchConfiguration(ns_key, ns_norm)]
