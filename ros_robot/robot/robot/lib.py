import math
from .config_api import *
from .API_func import *

# return MergedPoint closest to (posX, posY)
def find_closest_merged_point(posX, posY, merged_points):
    if not merged_points:
        return None

    closest_mp = None
    min_distance = float("inf")

    for mp in merged_points:
        tp = mp.target

        dx = posX - tp.posX
        dy = posY - tp.posY
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < min_distance:
            min_distance = distance
            closest_mp = mp

    return closest_mp

#merged_point_list : list of MergedPoint
#name : string corresponding to pointName
#return uuid, pointId, posX, posY
def point_info_from_name(name, merged_point_list):
    for mp in merged_point_list:
        if mp.point.pointName == name:
            return (
                mp.point.uuid,
                mp.point.pointId,
                mp.target.posX,
                mp.target.posY
            )
    return None, None, None, None

#keep only TargetPoints with type 'normal' (remove charging pile and origin)
def filter_normal_targets(target_list):
    target_list[:] = [tp for tp in target_list if tp.type == "normal"]
    return

#sort points in 4 lists based on group names defined during mapping phase and orientation
def sort_merged_points(merged_points):
    left_forward = []
    left_backward = []
    right_forward = []
    right_backward = []

    for mp in merged_points:
        area = mp.point.area
        tp = mp.target

        yaw = quaternion_to_yaw(
            tp.quatX,
            tp.quatY,
            tp.quatZ,
            tp.quatW
        )

        # forward: 0 → pi
        is_forward = yaw >= 0

        if area == "left":
            if is_forward:
                left_forward.append(mp)
            else:
                left_backward.append(mp)

        elif area == "right":
            if is_forward:
                right_forward.append(mp)
            else:
                right_backward.append(mp)

    return left_backward, left_forward, right_backward, right_forward

#merge list of Point and list of TargetPoint
def merge_points(points, target_points):
    target_by_name = {tp.name: tp for tp in target_points if tp.name is not None}

    return [
        MergedPoint(point=p, target=target_by_name[p.pointName])
        for p in points
        if p.pointName in target_by_name
    ]

def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

def pi_bracket(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


