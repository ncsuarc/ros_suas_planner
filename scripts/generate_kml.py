#!/usr/bin/env python3
from ros_suas_planner.interop_loader import InteropLoader
from ros_interop.msg import Mission
import rospy
from simplekml import Kml, AltitudeMode, MultiGeometry, Color, Polygon
from polycircles.polycircles import Polycircle
from pathlib import Path
from math import sqrt,atan,pi
from pyproj import Geod


def generate_kml(interop_loader: InteropLoader, mission_folder: Path) -> Kml:
    mission_folder = mission_folder.joinpath(f"Mission_{interop_loader.get_mission_id()}")
    mission_folder.mkdir()

    kml = Kml()
    
    lost_comms = interop_loader.get_lost_comms()
    kml.newpoint(name="Lost Comms Point", coords=[(lost_comms.latitude, lost_comms.longitude)])

    fly_zone = interop_loader.get_fly_zone()
    fly_zone_bounds = []
    for b in fly_zone.boundary_points:
        fly_zone_bounds.append((b.latitude, b.longitude, fly_zone.altitude_max))
    fly_zone_poly: Polygon = kml.newpolygon(name="Fly Zone", outerboundaryis=fly_zone_bounds, altitudemode=AltitudeMode.relativetoground, extrude=1)
    fly_zone_poly.polystyle.color = Color.red

    waypoints = interop_loader.get_waypoints()
    waypoints_kml: MultiGeometry = kml.newmultigeometry(name="Waypoints")
    waypoints_kml.style.iconstyle.color = Color.blue
    for i, w in enumerate(waypoints):
        waypoints_kml.newpoint(name=f"Waypoint {i}", coords=[(w.latitude, w.longitude, w.altitude)], altitudemode=AltitudeMode.relativetoground)

    search_grid = interop_loader.get_search_grid()
    search_grid_bounds = []
    for b in search_grid:
        search_grid_bounds.append((b.latitude, b.longitude))
    search_grid_poly: Polygon = kml.newpolygon(name="Search Grid", outerboundaryis=search_grid_bounds)
    search_grid_poly.polystyle.color = Color.red

    off_axis = interop_loader.get_off_axis()
    kml.newpoint(name="Off Axis Target", coords=[(off_axis.latitude, off_axis.longitude, 0)], altitudemode=AltitudeMode.relativetoground)
    
    emergent = interop_loader.get_emergent_last_known()
    kml.newpoint(name="Emergent Last Known Position", coords=[(emergent.latitude, emergent.longitude, 0)], altitudemode=AltitudeMode.relativetoground)

    drop_kml: MultiGeometry = kml.newmultigeometry(name="Drop")
    drop_kml.polystyle.color = Color.purple
    drop_kml.style.iconstyle.color = Color.purple

    air_drop = interop_loader.get_air_drop_boundary()
    air_drop_bounds = []
    for b in air_drop:
        air_drop_bounds.append((b.latitude, b.longitude))
    drop_kml.newpolygon(name="Air Drop Boundary", outerboundaryis=air_drop_bounds)
    
    air_drop_point = interop_loader.get_air_drop_point()
    drop_kml.newpoint(name="Air Drop Point", coords=[(air_drop_point.latitude, air_drop_point.longitude, 0)], altitudemode=AltitudeMode.relativetoground)

    ugv_drive = interop_loader.get_ugv_drive_point()
    drop_kml.newpoint(name="UGV Drive Point", coords=[(ugv_drive.latitude, ugv_drive.longitude, 0)], altitudemode=AltitudeMode.relativetoground)

    obstacle_kml: MultiGeometry = kml.newmultigeometry(name="Obstacles")
    obstacle_kml.polystyle.color = Color.yellow
    obstacles = interop_loader.get_obstacles()
    for i, o in enumerate(obstacles):
        o_poly = Polycircle(o.latitude, o.longitude, o.radius * 0.3048, number_of_vertices=12)
        obstacle_kml.newpolygon(f"Obstacle {i}", outerboundaryis=o_poly.to_kml())

    geod = Geod(ellps='WGS84')
    map_center = interop_loader.get_map_center_point()
    map_height = interop_loader.get_map_height()
    map_width = map_height * 16 / 9
    map_diag = sqrt( map_width**2 + map_height**2 ) / 2
    azimuth1 = atan(map_width/map_height)
    azimuth2 = atan(-map_width/map_height)
    azimuth3 = azimuth1 + pi
    azimuth4 = azimuth2 + pi

    pt1_lon, pt1_lat, _ = geod.fwd(map_center.longitude, map_center.latitude, azimuth1*180/pi, map_diag)
    pt2_lon, pt2_lat, _ = geod.fwd(map_center.longitude, map_center.latitude, azimuth2*180/pi, map_diag)
    pt3_lon, pt3_lat, _ = geod.fwd(map_center.longitude, map_center.latitude, azimuth3*180/pi, map_diag)
    pt4_lon, pt4_lat, _ = geod.fwd(map_center.longitude, map_center.latitude, azimuth4*180/pi, map_diag)

    kml.newpolygon(name="Map Area", outerboundaryis=[(pt1_lon, pt1_lat), (pt2_lon, pt2_lat), (pt3_lon, pt3_lat), (pt4_lon, pt4_lat)])

    kml.save(mission_folder.joinpath("General.kml").resolve())


if __name__ == "__main__":
    rospy.init_node("suas_planner")
    mission_id = rospy.get_param("/ros_suas_planner/param/mission_id", 0)
    mission_folder = rospy.get_param("/ros_suas_planner/param/mission_folder", "/RAID/Mission")
    mission_mode = rospy.get_param("/ros_suas_planner/param/mission_mode", "Testing")
    mission_folder = Path(mission_folder).joinpath(mission_mode)
    mission_folder.mkdir(parents=True)

    rospy.wait_for_service("/judges_server/mission")
    mission_service = rospy.ServiceProxy("/judges_server/mission", Mission)
    interop_loader = InteropLoader(mission_service, mission_id)

    generate_kml(interop_loader, mission_folder)