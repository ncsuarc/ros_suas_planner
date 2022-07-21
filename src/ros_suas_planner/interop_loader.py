
import rospy
from ros_interop.srv import MissionRequest
from ros_interop.msg import RequestType, FlyZone, StationaryObstacle, mission
from geographic_msgs.msg import GeoPoint


from typing import List

class InteropLoader(object):

    def __init__(self, mission_proxy:rospy.ServiceProxy, id:int) -> None:
        self.mission_proxy = mission_proxy
        self.mission: mission = None
        if id is not None:
            self.get_mission(id)

    def get_mission(self, id: int) -> None:
        mission_req = MissionRequest()
        mission_req.mission_id = id
        mission_req.request_type = RequestType(RequestType.GET)
        self.mission = self.mission_proxy(mission_req).mission_info

    def get_mission_id(self) -> int:
        return self.mission.id if self.mission is not None else None

    def get_lost_comms(self) -> GeoPoint:
        return self.mission.lost_comms_pos if self.mission is not None else None

    def get_fly_zone(self) -> List[FlyZone]:
        return self.mission.fly_zones if self.mission is not None else None

    def get_waypoints(self) -> List[GeoPoint]:
        return self.mission.waypoints if self.mission is not None else None

    def get_search_grid(self) -> List[GeoPoint]:
        return self.mission.search_grid_points if self.mission is not None else None

    def get_off_axis(self) -> GeoPoint:
        return self.mission.off_axis_odlc_pos if self.mission is not None else None

    def get_emergent_last_known(self) -> GeoPoint:
        return self.mission.emergent_last_known_pos if self.mission is not None else None
    
    def get_air_drop_boundary(self) -> List[GeoPoint]:
        return self.mission.air_drop_boundary_points if self.mission is not None else None

    def get_air_drop_point(self) -> GeoPoint:
        return self.mission.air_drop_pos if self.mission is not None else None

    def get_ugv_drive_point(self) -> GeoPoint:
        return self.mission.ugv_drive_pos if self.mission is not None else None

    def get_obstacles(self) -> List[StationaryObstacle]:
        return self.mission.stationary_obstacles if self.mission is not None else None

    def get_map_center_point(self) -> GeoPoint:
        return self.mission.map_center_pos if self.mission is not None else None

    def get_map_height(self) -> float:
        return self.mission.map_height if self.mission is not None else None
