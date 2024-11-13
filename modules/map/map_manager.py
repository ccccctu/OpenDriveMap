# map_manager.py

from lxml import etree
from modules.basics.open_drive import OpenDrive, Header
from modules.basics.junction import Junction, Connection as JunctionConnection, LaneLink as JunctionConnectionLaneLink
from modules.basics.road import Road, Predecessor as RoadLinkPredecessor, Successor as RoadLinkSuccessor, Neighbor as RoadLinkNeighbor, Type as RoadType, Speed as RoadTypeSpeed
from modules.basics.lane import LaneOffset as RoadLanesLaneOffset, Lane as RoadLaneSectionLane, LaneSection as RoadLanesSection, LaneWidth as RoadLaneSectionLaneWidth, LaneBorder as RoadLaneSectionLaneBorder
import numpy as np
import math
import copy
import json

class MapManager:
    def __init__(self, xml_file_path):
        self._OpenDrive = None
        self._lane_net = None
        self.load_OpenDrive(xml_file_path)
        self.build_map()

    @property
    def OpenDrive(self):
        return copy.deepcopy(self._OpenDrive)

    @property
    def lane_net(self):
        return copy.deepcopy(self._lane_net)

    def load_OpenDrive(self, xml_file_path):
        """载入XML文件并解析为 OpenDrive 对象"""
        try:
            import os
            if not os.path.exists(xml_file_path):
                print(f"File '{xml_file_path}' does not exist.")
                return False
            tree = etree.parse(xml_file_path)
            root = tree.getroot()
            self._OpenDrive = self.parse_OpenDrive(root)
            print("OpenDrive loaded successfully.")
            return True
        except Exception as e:
            print(f"Failed to load OpenDrive: {e}")
            return False

    def get_road(self, road_id):
        road = self._OpenDrive.getRoad(road_id)
        return road
    
    def get_lane_section(self, road_id, lane_section_id):
        road = self.get_road(road_id)
        lane_section = road.lanes.getLaneSection(lane_section_id)
        return lane_section

    def get_lane(self, road_id, lane_section_id, lane_id):
        lane_section = self.get_lane_section(road_id, lane_section_id)
        lane = lane_section.getLane(lane_id)
        return lane

    def road_exist(self, road_id):
        """Check if the specified road exists."""
        road = self._OpenDrive.getRoad(road_id)
        if road is None:
            print(f"[MapManager] Info - Specified road ID not found: {road_id}")
            return False
        
        return True
    
    def lane_section_exist(self, road_id, lane_section_id):
        """Check if the specified lane section exists within the road."""
        if not self.road_exist(road_id):
            return False
        lane_section = self.get_lane_section(road_id, lane_section_id)
        if lane_section is None:
            print(f"[MapManager] Info - Specified lane section ID not found: {lane_section_id} in road ID: {road_id}")
            return False
        
        return True
    
    def lane_exist(self, road_id, lane_section_id, lane_id):
        """Check if the specified lane exists within the lane section and road."""
        if not self.road_exist(road_id):
            return False
        if not self.lane_section_exist(road_id, lane_section_id):
            return False
        lane = self.get_lane(road_id, lane_section_id, lane_id)
        if lane is None:
            print(f"[MapManager] Info - Specified lane ID not found: {lane_id} in lane section ID: {lane_section_id} and road ID: {road_id}")
            return False

        return True

    def find_position(self, x, y):
        """Find the position (x, y) in the road network."""
        min_disition = -1
        road_id = None
        lane_section_id = None
        lane_id = None
        offset = None
        x1 = None
        y1 = None
        for road in self._OpenDrive.roads:
            lanes = road.lanes
            lane_sections = lanes.laneSections
            for i, lane_section in enumerate(lane_sections):
                left_lanes = lane_section.leftLanes
                right_lanes = lane_section.rightLanes
                for j, lane in enumerate(left_lanes, start=1):
                    if lane.type == "driving":
                        center_line = lane.center_line
                        for point in center_line:
                            xx = x - point["position"][0]
                            yy = y - point["position"][1]
                            dis = xx ** 2 + yy ** 2
                            if min_disition == -1 or dis < min_disition:
                                x1, y1 = point["position"]
                                road_id = road.id
                                lane_section_id = i
                                lane_id = j
                                offset = center_line[0]["s_geometry"] - point["s_geometry"]
                                min_disition = dis
                for j, lane in enumerate(right_lanes, start=1):
                    if lane.type == "driving":
                        center_line = lane.center_line
                        for point in center_line:
                            xx = x - point["position"][0]
                            yy = y - point["position"][1]
                            dis = xx ** 2 + yy ** 2
                            if min_disition == -1 or dis < min_disition:
                                x1, y1 = point["position"]
                                road_id = road.id
                                lane_section_id = i
                                lane_id = j
                                offset = center_line[0]["s_geometry"] - point["s_geometry"]
                                min_disition = dis
        position = Position(road_id, lane_section_id, lane_id, offset)
        return position

    def build_lanelink_between_two_lane(self, 
                                        start_road_id, 
                                        start_lane_section_id,  
                                        start_lane_id,
                                        end_road_id,
                                        end_lane_section_id,
                                        end_lane_id):
        """Create a link between two lanes."""
        if not self.lane_exist(start_road_id, start_lane_section_id, start_lane_id):
            print(f"[MapManager] Info - Start lane not found for connection: ({start_road_id}, {start_lane_section_id}, {start_lane_id})")
            return False
        if not self.lane_exist(end_road_id, end_lane_section_id, end_lane_id):
            print(f"[MapManager] Info - End lane not found for connection: ({end_road_id}, {end_lane_section_id}, {end_lane_id})")
            return False
        
        start_lane = self.get_lane(start_road_id, start_lane_section_id, start_lane_id)
        end_lane = self.get_lane(end_road_id, end_lane_section_id, end_lane_id)
        start_lane_unique_id = start_lane.unique_id
        end_lane_unique_id = end_lane.unique_id

        # Establish connection
        start_node = self._OpenDrive.getNode(start_lane_unique_id)
        start_node.add_edge(end_lane_unique_id)
        print(f"[MapManager] Info - Successfully created link: {start_lane_unique_id} (road:{start_road_id}, lane_section:{start_lane_section_id}, lane:{start_lane_id}) -> {end_lane_unique_id} (road:{end_road_id}, lane_section:{end_lane_section_id}, lane:{end_lane_id})")
        return True
    
    def build_lanelink_between_two_road(self, 
                                        incoming_road_id, 
                                        connecting_road_id, 
                                        contact_point, 
                                        from_id, 
                                        to_id):
        """Create a link between two roads."""
        if contact_point == "start":
            if from_id > 0 and to_id > 0:
                start_road_id = connecting_road_id
                start_lane_section_id = 0
                start_lane_id = to_id
                end_road_id = incoming_road_id
                end_lane_section_id = self.get_road(end_road_id).lanes.getLastLaneSectionIdx()
                end_lane_id = from_id
            elif from_id < 0 and to_id < 0:
                start_road_id = incoming_road_id
                start_lane_section_id = self.get_road(start_road_id).lanes.getLastLaneSectionIdx()
                start_lane_id = from_id
                end_road_id = connecting_road_id
                end_lane_section_id = 0
                end_lane_id = to_id
            elif from_id > 0 and to_id < 0:
                start_road_id = incoming_road_id
                start_lane_section_id = 0
                start_lane_id = from_id
                end_road_id = connecting_road_id
                end_lane_section_id = 0
                end_lane_id = to_id
            elif from_id < 0 and to_id > 0:
                start_road_id = connecting_road_id
                start_lane_section_id = 0
                start_lane_id = to_id
                end_road_id = incoming_road_id
                end_lane_section_id = 0
                end_lane_id = from_id
        else:
            if from_id > 0 and to_id > 0:
                start_road_id = incoming_road_id
                start_lane_section_id = 0
                start_lane_id = from_id
                end_road_id = connecting_road_id
                end_lane_section_id = self.get_road(end_road_id).lanes.getLastLaneSectionIdx()
                end_lane_id = to_id
            elif from_id < 0 and to_id < 0:
                start_road_id = connecting_road_id
                start_lane_section_id = self.get_road(start_road_id).lanes.getLastLaneSectionIdx()
                start_lane_id = to_id
                end_road_id = incoming_road_id
                end_lane_section_id = 0
                end_lane_id = from_id
            elif from_id > 0 and to_id < 0:
                start_road_id = connecting_road_id
                start_lane_section_id = 0
                start_lane_id = to_id
                end_road_id = incoming_road_id
                end_lane_section_id = 0
                end_lane_id = from_id
            elif from_id < 0 and to_id > 0:
                start_road_id = incoming_road_id
                start_lane_section_id = 0
                start_lane_id = from_id
                end_road_id = connecting_road_id
                end_lane_section_id = 0
                end_lane_id = to_id
        return self.build_lanelink_between_two_lane(start_road_id, start_lane_section_id,  
                                    start_lane_id, end_road_id, end_lane_section_id, end_lane_id)


    def build_one_side_link(self, road_id, lane_section_id, left):
        """Create lane link relationships for the specified road and lane section, handling left or right lanes."""
        road = self._OpenDrive.getRoad(road_id)
        road_predecessor = road.link.predecessor
        road_successor = road.link.successor
        lanes = (road.lanes.getLaneSection(lane_section_id).leftLanes if left else 
                 road.lanes.getLaneSection(lane_section_id).rightLanes)

        for lane in lanes:
            lane_id = lane.id
            lane_predecessor_id = lane.link.predecessorId
            lane_successor_id = lane.link.successorId

            # Handle predecessor lane connection
            if lane_predecessor_id is not None:
                if lane_section_id == 0:
                    if road_predecessor is None:
                        print(f"[MapManager] Info - No predecessor road found for road: {road_id}")
                        return False
                    if road_predecessor.elementType == "road":
                        if not self.build_lanelink_between_two_road(road_id, 
                                                                    road_predecessor.elementId, 
                                                                    road_predecessor.contactPoint, 
                                                                    lane_id, 
                                                                    lane_predecessor_id):
                            return False
                else:
                    if left:
                        if not self.build_lanelink_between_two_lane(road_id, 
                                                                    lane_section_id, 
                                                                    lane_id,
                                                                    road_id, 
                                                                    lane_section_id - 1, 
                                                                    lane_predecessor_id):
                            return False
                    else:
                        if not self.build_lanelink_between_two_lane(road_id, 
                                                                    lane_section_id - 1, 
                                                                    lane_predecessor_id,
                                                                    road_id, 
                                                                    lane_section_id, 
                                                                    lane_id):
                            return False
            # Handle successor lane connection
            if lane_successor_id is not None:
                if lane_section_id == road.lanes.getLastLaneSectionIdx():
                    if road_successor is None:
                        print(f"[MapManager] Info - No successor road found for road: {road_id}")
                        return False
                    if road_successor.elementType == "road":
                        if not self.build_lanelink_between_two_road(road_id, 
                                                                    road_successor.elementId, 
                                                                    road_successor.contactPoint, 
                                                                    lane_id, 
                                                                    lane_successor_id):
                            return False
                else:
                    if left:
                        if not self.build_lanelink_between_two_lane(road_id, 
                                                                    lane_section_id + 1, 
                                                                    lane_successor_id,
                                                                    road_id, 
                                                                    lane_section_id, 
                                                                    lane_id):
                            return False
                    else:
                        if not self.build_lanelink_between_two_lane(road_id, 
                                                                    lane_section_id, 
                                                                    lane_id,
                                                                    road_id, 
                                                                    lane_section_id + 1, 
                                                                    lane_successor_id):
                            return False
        return True
    
    def build_map(self):
        """[MapManager]Info - Initiates the construction of road network relationships within the map."""

        print("[MapManager]Info - Starting to build road network relationships...")

        # Iterate over all roads in OpenDrive data
        for road in self._OpenDrive.roads:
            road_id = road.id
            Junction_id = road.junction
            lane_sections = road.lanes.laneSections
            # Build links for each lane section in both directions
            for lane_section_id, lane_section in enumerate(lane_sections):
                left = True
                right = False
                if not self.build_one_side_link(road_id, lane_section_id, left) or \
                   not self.build_one_side_link(road_id, lane_section_id, right):
                    print(f"[MapManager]Error - Failed to establish lane links for road {road_id}, lane section {lane_section_id}")
                    return False

        # Iterate over all junctions in OpenDrive data
        for junction in self._OpenDrive.junctions:
            connections = junction.connections
            
            # Process each connection within the junction
            for connection in connections:
                incoming_road_id = connection.incomingRoad
                connecting_road_id = connection.connectingRoad
                contact_point = connection.contactPoint
                lane_links = connection.laneLinks
                # Build lane links between roads at the junction
                for lane_link in lane_links:
                    from_id = lane_link.fromId
                    to_id = lane_link.toId
                    if not self.build_lanelink_between_two_road(incoming_road_id,
                                                                connecting_road_id,
                                                                contact_point,
                                                                from_id,
                                                                to_id):
                        print(f"[MapManager]Error - Failed to link lanes between incoming road {incoming_road_id} and connecting road {connecting_road_id}")
                        return False
        
        self._lane_net = self._OpenDrive.nodes
        print("[MapManager]Info - Road network relationships successfully established.")
        return True
    
    def calculate_one_side_line(self, lanes, left, ref_points, lane_section_start_s, lane_section_end_s):
        """ Calculate lane geometry for one side (left or right) within a lane section, including left line、right line and center line. """
        lane_num = len(lanes)
        if lane_num == 0:
            return
        start = True
        end = False
        reverse = True
        no_reverse = False
        
        start_index_of_lane_section = self.find_index(ref_points, lane_section_start_s, no_reverse, start)
        end_index_of_lane_section = self.find_index(ref_points, lane_section_end_s, no_reverse, end)

        lane_section_points = copy.deepcopy(ref_points[start_index_of_lane_section:end_index_of_lane_section + 1])
        center_line = copy.deepcopy(lane_section_points)
        for i, lane in enumerate(lanes, start=1):
            widths = lane.widths
            width_num = len(widths)
            offset_ratio = 0.5
            if i == 1 and left:
                lane_section_points.reverse()
                center_line.reverse()
            lane.setLeftLine(lane_section_points)

            for j, width in enumerate(widths):
                width_start_s = width.sOffset + lane_section_start_s
                if j == width_num - 1:
                    width_end_s = lane_section_end_s
                else:
                    width_end_s = widths[j + 1].sOffset + lane_section_start_s
                if left:
                    if j == 0:
                        start_index = self.find_index(lane_section_points, width_end_s, reverse, start)
                        end_index = self.find_index(lane_section_points, width_start_s, reverse, end)
                    else:
                        end_index = start_index - 1
                        start_index = self.find_index(lane_section_points, width_end_s, reverse, start)
                else:
                    if j == 0:
                        start_index = self.find_index(lane_section_points, width_start_s, no_reverse, start)
                        end_index = self.find_index(lane_section_points, width_end_s, no_reverse, end)
                    else:
                        start_index = end_index + 1
                        end_index = self.find_index(lane_section_points, width_end_s, no_reverse, end)
                nominal_width = lane.getNormalWidth()
                a, b, c, d = width.a, width.b, width.c, width.d
                lane_section_points_copy1 = copy.deepcopy(lane_section_points)
                lane_section_points_copy2 = copy.deepcopy(lane_section_points)
                self.calculate_offset(lane_section_points_copy1, center_line, start_index, end_index, 
                                      offset_ratio * a, offset_ratio * b, offset_ratio * c, offset_ratio * d, width_start_s, no_reverse, left)
                self.calculate_offset(lane_section_points_copy2, lane_section_points, start_index, end_index, 
                                      a, b, c, d, width_start_s, no_reverse, left, 0.5)
            lane.setRightLine(lane_section_points)
            lane.setCenterLine(center_line)

    def calculate_lane_line_and_center_line(self):
        """ Calculate the reference line, center divider, and associated lines for each road and lane. """
        for road in self._OpenDrive.roads:
            lanes = road.lanes
            ref_points = []
            seen_s_road = set()

            geometries =  copy.deepcopy(road.planView.geometries)
            for geometrie in geometries:
                for point in geometrie.getRefPoints():
                    s_road = point['s_road']
                    if s_road not in seen_s_road:
                        ref_points.append(point)
                        seen_s_road.add(s_road)
            road.setRefPoints(ref_points)

            # Calculate the reference line, center divider, and associated lines for each road and lane
            lane_offsets = lanes.laneOffsets
            if lane_offsets:
                lane_offsets_size = len(lane_offsets)
                for i, lane_offset in enumerate(lane_offsets):
                    start_s = lane_offset.sPos
                    if i == lane_offsets_size - 1:
                        end_s = road.length
                    else:
                        end_s = lane_offsets[i + 1].sPos
                    start = True
                    end = False
                    left = True
                    no_reverse = False
                    if i == 0:
                        start_index = self.find_index(ref_points, start_s, no_reverse, start)
                        end_index = self.find_index(ref_points, end_s, no_reverse, end)
                    else:
                        start_index = end_index + 1
                        end_index = self.find_index(ref_points, end_s, no_reverse, end)
                    a = lane_offset.a
                    b = lane_offset.b
                    c = lane_offset.c
                    d = lane_offset.d
                    ref_points_copy = copy.deepcopy(ref_points)
                    self.calculate_offset(ref_points_copy, ref_points, start_index, end_index, 
                                          a, b, c, d, start_s, no_reverse, left)
                    road.setCenterLine(ref_points)

            # Calculate the lane lines for each lane section on both left and right sides
            for i, lane_section in enumerate(lanes.laneSections):
                lane_section_s = lane_section.sPos
                left_lanes = lane_section.leftLanes
                right_lanes = lane_section.rightLanes
                left = True
                right = False
                if i == len(lanes.laneSections) - 1:
                    lane_section_end_s = road.length
                else:
                    lane_section_end_s = lanes.getLaneSection(i + 1).sPos
                self.calculate_one_side_line(left_lanes, left, ref_points, lane_section_s, lane_section_end_s)
                self.calculate_one_side_line(right_lanes, right, ref_points, lane_section_s, lane_section_end_s)


    def calculate_offset(self, ref_points, res_points, start_index, end_index, a, b, c, d,
                          offset, reverse=False, left=True, threshold=-1):
        """ Calculate the offset line. """
        
        theta = -math.pi / 2
        if left:
            theta = -theta
        indices = range(start_index, end_index + 1)
        if reverse:
            indices = range(end_index, start_index - 1, -1)

        for i, j in enumerate(indices):
            p = ref_points[j]
            s = p["s_geometry"] - offset
            dis = a + b * s + c * s ** 2 + d * s ** 3

            x, y = p["position"]
            tangent = p["tangent"]
            new_x = x + dis * math.cos(tangent + theta)
            new_y = y + dis * math.sin(tangent + theta)

            res_points[start_index + i]["position"] = (new_x, new_y)
            res_points[start_index + i]["s_geometry"] = p["s_geometry"]
            res_points[start_index + i]["tangent"] = tangent

    def find_index(self, ref_points, s, reverse, start):
        """ Find the closest point to a given distance 's' in the reference points list. """
        # start=True returns the first point on the right of 's'.
        # start=False returns the first point on the left of 's'.
        start_index = 0
        end_index = len(ref_points) - 1
        while (start_index <= end_index):
            mid_index = start_index + (end_index - start_index) // 2
            if abs(ref_points[mid_index]['s_road'] - s) <= 1e-5:
                return mid_index
            elif (ref_points[mid_index]['s_road'] - s > 1e-5 and reverse) or \
                 (s - ref_points[mid_index]['s_road'] > 1e-5 and not reverse):
                start_index = mid_index + 1
            else:
                end_index = mid_index - 1
        if start:
            if start_index == len(ref_points):
                start_index = len(ref_points) - 1
            elif start_index == -1:
                start_index = 0
            return start_index
        if end_index == len(ref_points):
            end_index = len(ref_points) - 1
        elif start_index == -1:
            end_index = 0
        return end_index

    def parse_OpenDrive(self, rootNode):
        """ Parse XML, get OpenDrive"""
        if not etree.iselement(rootNode):
            raise TypeError("Argument rootNode is not a xml element")
        newOpenDrive = OpenDrive()

        # Header
        header = rootNode.find("header")
        if header is not None:
            newOpenDrive._header = Header()
            newOpenDrive._header._revMajor = header.get("revMajor")
            newOpenDrive._header._revMinor = header.get("revMinor")
            newOpenDrive._header._name = header.get("name")
            newOpenDrive._header._version = header.get("version")
            newOpenDrive._header._date = header.get("date")
            newOpenDrive._header._north = header.get("north")
            newOpenDrive._header._south = header.get("south")
            newOpenDrive._header._east = header.get("east")
            newOpenDrive._header._west = header.get("west")
            print("Header loaded successfully.")
            if header.find("geoReference") is not None:
                pass
        
        # Junctions
        for junction in rootNode.findall("junction"):
            newJunction = Junction()
            newJunction.id = int(junction.get("id"))
            newJunction.name = str(junction.get("name"))
            for connection in junction.findall("connection"):
                newConnection = JunctionConnection()
                newConnection.id = connection.get("id")
                newConnection.incomingRoad = connection.get("incomingRoad")
                newConnection.connectingRoad = connection.get("connectingRoad")
                newConnection.contactPoint = connection.get("contactPoint")
                for laneLink in connection.findall("laneLink"):
                    newLaneLink = JunctionConnectionLaneLink()
                    newLaneLink.fromId = laneLink.get("from")
                    newLaneLink.toId = laneLink.get("to")
                    newConnection.addLaneLink(newLaneLink)
                newJunction.addConnection(newConnection)
            newOpenDrive.junctions.append(newJunction)

        # Load roads
        for road in rootNode.findall("road"):
            newRoad = Road()
            newRoad.id = int(road.get("id"))
            newRoad.name = road.get("name")
            newRoad.junction = int(road.get("junction")) if road.get("junction") != "-1" else None
            newRoad.length = float(road.get("length"))

            # Links
            if road.find("link") is not None:
                predecessor = road.find("link").find("predecessor")
                if predecessor is not None:
                    newPredecessor = RoadLinkPredecessor()
                    newPredecessor.elementType = predecessor.get("elementType")
                    newPredecessor.elementId = predecessor.get("elementId")
                    newPredecessor.contactPoint = predecessor.get("contactPoint")
                    newRoad.link.predecessor = newPredecessor

                successor = road.find("link").find("successor")
                if successor is not None:
                    newSuccessor = RoadLinkSuccessor()
                    newSuccessor.elementType = successor.get("elementType")
                    newSuccessor.elementId = successor.get("elementId")
                    newSuccessor.contactPoint = successor.get("contactPoint")
                    newRoad.link.successor = newSuccessor

                for neighbor in road.find("link").findall("neighbor"):
                    newNeighbor = RoadLinkNeighbor()
                    newNeighbor.side = neighbor.get("side")
                    newNeighbor.elementId = neighbor.get("elementId")
                    newNeighbor.direction = neighbor.get("direction")
                    newRoad.link.neighbors.append(newNeighbor)

            # Type
            for roadType in road.findall("type"):
                newType = RoadType()
                newType.sPos = roadType.get("s")
                newType.type = roadType.get("type")
                if roadType.find("speed"):
                    newSpeed = RoadTypeSpeed()
                    newSpeed.max = roadType.find("speed").get("max")
                    newSpeed.unit = roadType.find("speed").get("unit")
                    newType.speed = newSpeed
                newRoad.types.append(newType)

            # Plan view
            for geometry in road.find("planView").findall("geometry"):
                startCoord = [float(geometry.get("x")), float(geometry.get("y"))]
                if geometry.find("line") is not None:
                    newRoad.planView.addLine(float(geometry.get("s")), startCoord, float(geometry.get("hdg")), float(geometry.get("length")))
                elif geometry.find("spiral") is not None:
                    newRoad.planView.addSpiral(float(geometry.get("s")), startCoord, float(geometry.get("hdg")), float(geometry.get("length")),
                                                float(geometry.find("spiral").get("curvStart")), 
                                                float(geometry.find("spiral").get("curvEnd")))
                elif geometry.find("arc") is not None:
                    newRoad.planView.addArc(float(geometry.get("s")), startCoord, float(geometry.get("hdg")), float(geometry.get("length")), float(geometry.find("arc").get("curvature")))
                elif geometry.find("poly3") is not None:
                    raise NotImplementedError()
                elif geometry.find("paramPoly3") is not None:
                    if geometry.find("paramPoly3").get("pRange"):
                        if geometry.find("paramPoly3").get("pRange") == "arcLength":
                            pMax = float(geometry.get("length"))
                        else:
                            pMax = None
                    else:
                        pMax = None
                    newRoad.planView.addParamPoly3( \
                        float(geometry.get("s")), \
                        startCoord, \
                        float(geometry.get("hdg")), \
                        float(geometry.get("length")), \
                        float(geometry.find("paramPoly3").get("aU")), \
                        float(geometry.find("paramPoly3").get("bU")), \
                        float(geometry.find("paramPoly3").get("cU")), \
                        float(geometry.find("paramPoly3").get("dU")), \
                        float(geometry.find("paramPoly3").get("aV")), \
                        float(geometry.find("paramPoly3").get("bV")), \
                        float(geometry.find("paramPoly3").get("cV")), \
                        float(geometry.find("paramPoly3").get("dV")), \
                        pMax \
                    )
                else:
                    raise Exception("invalid xml")

            # Lanes
            lanes = road.find("lanes")
            if lanes is None:
                raise Exception("Road must have lanes element")
            
            # Lane offset
            for laneOffset in lanes.findall("laneOffset"):
                newLaneOffset = RoadLanesLaneOffset()
                newLaneOffset.sPos = laneOffset.get("s")
                newLaneOffset.a = laneOffset.get("a")
                newLaneOffset.b = laneOffset.get("b")
                newLaneOffset.c = laneOffset.get("c")
                newLaneOffset.d = laneOffset.get("d")
                newRoad.lanes.laneOffsets.append(newLaneOffset)

            # Lane sections
            for laneSectionIdx, laneSection in enumerate(road.find("lanes").findall("laneSection")):
                newLaneSection = RoadLanesSection()
                newLaneSection.idx = laneSectionIdx
                newLaneSection.sPos = laneSection.get("s")
                newLaneSection.singleSide = laneSection.get("singleSide")
                sides = dict(
                    left=newLaneSection.leftLanes,
                    center=newLaneSection.centerLanes,
                    right=newLaneSection.rightLanes
                    )
                for sideTag, newSideLanes in sides.items():
                    side = laneSection.find(sideTag)
                    if side is None:
                        continue
                    for lane in side.findall("lane"):
                        newLane = RoadLaneSectionLane()
                        newLane.id = lane.get("id")
                        newLane.type = lane.get("type")
                        newLane.level = lane.get("level")
                        newOpenDrive.addNode(Node(newRoad.id, newLaneSection.idx, newLane.id))
                        # Lane Links
                        if lane.find("link") is not None:
                            if lane.find("link").find("predecessor") is not None:
                                newLane.link.predecessorId = lane.find("link").find("predecessor").get("id")
                            if lane.find("link").find("successor") is not None:
                                newLane.link.successorId = lane.find("link").find("successor").get("id")
                        # Width
                        for widthIdx, width in enumerate(lane.findall("width")):
                            newWidth = RoadLaneSectionLaneWidth()
                            newWidth.idx = widthIdx
                            newWidth.sOffset = width.get("sOffset")
                            newWidth.a = width.get("a")
                            newWidth.b = width.get("b")
                            newWidth.c = width.get("c")
                            newWidth.d = width.get("d")
                            newLane.widths.append(newWidth)
                        # Border
                        for borderIdx, border in enumerate(lane.findall("border")):
                            newBorder = RoadLaneSectionLaneBorder()
                            newBorder.idx = borderIdx
                            newBorder.sPos = border.get("sOffset")
                            newBorder.a = border.get("a")
                            newBorder.b = border.get("b")
                            newBorder.c = border.get("c")
                            newBorder.d = border.get("d")
                            newLane.borders.append(newBorder)
                        # Road Marks
                        # TODO
                        # Material
                        # TODO
                        # Visiblility
                        # TODO
                        # Speed
                        # TODO
                        # Access
                        # TODO
                        # Lane Height
                        # TODO
                        # Rules
                        # TODO
                        newSideLanes.append(newLane)
                newRoad.lanes.laneSections.append(newLaneSection)

            for laneSection in newRoad.lanes.laneSections:
                if laneSection.idx + 1 >= len(newRoad.lanes.laneSections):
                    laneSection.length = newRoad.planView.getLength() - laneSection.sPos
                else:
                    laneSection.length = newRoad.lanes.laneSections[laneSection.idx + 1].sPos - laneSection.sPos

            for laneSection in newRoad.lanes.laneSections:
                for lane in laneSection.allLanes:
                    widthsPoses = np.array([x.sOffset for x in lane.widths] + [laneSection.length])
                    widthsLengths = widthsPoses[1:] - widthsPoses[:-1]
                    for widthIdx, width in enumerate(lane.widths):
                        width.length = widthsLengths[widthIdx]

            for geometry in newRoad.planView.geometries:
                geometry.calcRefPoints(step=0.5)

            newOpenDrive.roads.append(newRoad)

        return newOpenDrive
   
    def export_map_data_for_visualize(self, filename="map_data.json"):
        """ Export the map data to the specified location. """
        map_data = {
            "roads": [
                {
                    "road_id": road.id,
                    "junction": road.junction,
                    "reference_points": {
                        "x": [float(point["position"][0]) for point in road.ref_points],
                        "y": [float(point["position"][1]) for point in road.ref_points]
                    },
                    "center_line": {
                        "x": [float(point["position"][0]) for point in road.center_line],
                        "y": [float(point["position"][1]) for point in road.center_line]
                    },
                    "laneSections": [
                        {
                            "section_id": lane_section.idx,
                            "left_lanes": [
                                {
                                    "lane_id": lane.id,
                                    "lane_type": lane.type,
                                    "left_line": {
                                        "x": [float(point["position"][0]) for point in lane.left_line],
                                        "y": [float(point["position"][1]) for point in lane.left_line]
                                    },
                                    "right_line": {
                                        "x": [float(point["position"][0]) for point in lane.right_line],
                                        "y": [float(point["position"][1]) for point in lane.right_line]
                                    }
                                }
                                for lane in lane_section.leftLanes
                            ],
                            "right_lanes": [
                                {
                                    "lane_id": lane.id,
                                    "lane_type": lane.type,
                                    "left_line": {
                                        "x": [float(point["position"][0]) for point in lane.left_line],
                                        "y": [float(point["position"][1]) for point in lane.left_line]
                                    },
                                    "right_line": {
                                        "x": [float(point["position"][0]) for point in lane.right_line],
                                        "y": [float(point["position"][1]) for point in lane.right_line]
                                    }
                                }
                                for lane in lane_section.rightLanes
                            ]
                        }
                        for lane_section in road.lanes.laneSections
                    ]
                }
                for road in self._OpenDrive.roads
            ]
        }

        json_str = json.dumps(map_data, indent=4)

        with open(filename, "w") as file:
            file.write(json_str)

        print(f"Data has been successfully exported to {filename}")

class Position:
    def __init__(self, road_id, lane_section_id, lane_id, offset):
        self.road_id = road_id
        self.lane_section_id = lane_section_id
        self.lane_id = lane_id
        self.offset = offset

class Edge:
    def __init__(self, id):
        self.id = id
        self.next = None

class Node:
    def __init__(self, road_id, lane_section_id, lane_id):
        self.road_id = road_id
        self.lane_section_id = lane_section_id
        self.lane_id = lane_id
        self.next = None

    def add_edge(self, edge_id):
        if self.next is None:
            self.next = Edge(edge_id)
            return
        
        current_edge = self.next
        while current_edge.next is not None:
            if current_edge.id == edge_id:
                return
            current_edge = current_edge.next

        if current_edge.id == edge_id:
            return
        current_edge.next = Edge(edge_id)

    def get_connections(self):
        connections = []
        current_edge = self.next
        while current_edge is not None:
            connections.append(current_edge.id)
            current_edge = current_edge.next
        return connections
    
    def print(self):
        print(f"Node: Road ID = {self.road_id}, Lane Section ID = {self.lane_section_id}, Lane ID = {self.lane_id}")
        current_edge = self.next
        if not current_edge:
            print("  No connections.")
        else:
            print("  Connections:")
            while current_edge is not None:
                print(f"    -> Edge to Lane ID {current_edge.id}")
                current_edge = current_edge.next
