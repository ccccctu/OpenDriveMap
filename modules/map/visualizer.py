# visualizer.py

import pygame
import json

class Visualizer:
    def __init__(self, map_file, width=1800, height=1100, area_select=10):
        with open(map_file, "r") as file:
            self.map_data = json.load(file)

        # Initialize Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Map Visualizer")
        self.clock = pygame.time.Clock()

        self.lane_info = []

        # Scaling and offset parameters
        self.scale = 4
        self.offset_x = 250
        self.offset_y = -280
        self.area_select = area_select

        # Color configurations
        self.DRIVING_COLOR = (135, 151, 154)
        self.TYPE_COLOR_DICT = {
            "shoulder": (136, 158, 131),
            "border": (84, 103, 80),
            "driving": self.DRIVING_COLOR,
            "stop": (128, 68, 59),
            "none": (236, 236, 236),
            "restricted": (165, 134, 88),
            "parking": self.DRIVING_COLOR,
            "median": (119, 155, 88),
            "biking": (108, 145, 125),
            "sidewalk": (106, 159, 170),
            "curb": (30, 49, 53),
            "exit": self.DRIVING_COLOR,
            "entry": self.DRIVING_COLOR,
            "onramp": self.DRIVING_COLOR,
            "offRamp": self.DRIVING_COLOR,
            "connectingRamp": self.DRIVING_COLOR,
            "onRamp": self.DRIVING_COLOR,
            "bidirectional": self.DRIVING_COLOR,
        }
        self.COLOR_CENTER_LANE = (255, 197, 0)
        self.COLOR_REFERENCE_LINE = (0, 0, 238)
        self.COLOR_NOMAL_LINE = (225, 225, 225)

    def transform_point(self, x, y):
        """ Transform map coordinates to screen coordinates. """
        screen_x = int(x * self.scale + self.offset_x)
        screen_y = int(-y * self.scale + self.offset_y)
        return screen_x, screen_y
    
    def draw_dashed_line(self, color, points, segment_length=5, gap_length=5, width=2):
        """ Draw a dashed line between points """
        i = 0
        while i < len(points):
            segment_end = min(i + segment_length, len(points))
            pygame.draw.lines(self.screen, color, False, points[i:segment_end], width)
            i += segment_length + gap_length
    
    def draw_map(self):
        """ Draw a map. """
        for road in self.map_data["roads"]:
            for lane_section in road["laneSections"]:
                for i, lane in enumerate(lane_section["left_lanes"]):
                    # road left lane area
                    lane_type = lane["lane_type"]
                    left_line_x = lane["left_line"]["x"]
                    left_line_y = lane["left_line"]["y"]
                    right_line_x = lane["right_line"]["x"]
                    right_line_y = lane["right_line"]["y"]
                    right_line_x_reversed = right_line_x[::-1]
                    right_line_y_reversed = right_line_y[::-1]
                    xs = left_line_x + right_line_x_reversed
                    ys = left_line_y + right_line_y_reversed
                    transformed_polygon_points = [self.transform_point(x, y) for x, y in zip(xs, ys)]
                    color = self.TYPE_COLOR_DICT[lane_type]
                    pygame.draw.polygon(self.screen, color, transformed_polygon_points, width=0)
                    self.lane_info.append({
                        "road_id": road["road_id"],
                        "lane_section_id": lane_section["section_id"],
                        "lane_id": lane["lane_id"],
                        "polygon": transformed_polygon_points
                    })
                    if road["junction"] == None:
                        # driving lane line
                        if abs(lane["lane_id"]) > 1 and lane["lane_type"] == "driving":
                            left_line_points = [self.transform_point(x, y) for x, y in zip(left_line_x, left_line_y)]
                            self.draw_dashed_line(self.COLOR_NOMAL_LINE, left_line_points)
                        # other lane line
                        elif abs(lane["lane_id"]) > 1 and lane["lane_type"] != "driving":
                            left_line_points = [self.transform_point(x, y) for x, y in zip(left_line_x, left_line_y)]
                            right_line_points = [self.transform_point(x, y) for x, y in zip(right_line_x, right_line_y)]
                            pygame.draw.lines(self.screen, self.COLOR_NOMAL_LINE, False, left_line_points, 3)
                            pygame.draw.lines(self.screen, self.COLOR_NOMAL_LINE, False, right_line_points, 3)
                for lane in lane_section["right_lanes"]:
                    # road right lane area
                    lane_type = lane["lane_type"]
                    left_line_x = lane["left_line"]["x"]
                    left_line_y = lane["left_line"]["y"]
                    right_line_x = lane["right_line"]["x"]
                    right_line_y = lane["right_line"]["y"]
                    right_line_x_reversed = right_line_x[::-1]
                    right_line_y_reversed = right_line_y[::-1]
                    xs = left_line_x + right_line_x_reversed
                    ys = left_line_y + right_line_y_reversed
                    transformed_polygon_points = [self.transform_point(x, y) for x, y in zip(xs, ys)]
                    color = self.TYPE_COLOR_DICT[lane_type]
                    pygame.draw.polygon(self.screen, color, transformed_polygon_points, width=0)
                    if road["junction"] == None:
                        # driving lane line
                        if abs(lane["lane_id"]) > 1 and lane["lane_type"] == "driving":
                            left_line_points = [self.transform_point(x, y) for x, y in zip(left_line_x, left_line_y)]
                            self.draw_dashed_line(self.COLOR_NOMAL_LINE, left_line_points)
                        # other lane line
                        elif abs(lane["lane_id"]) > 1 and lane["lane_type"] != "driving":
                            left_line_points = [self.transform_point(x, y) for x, y in zip(left_line_x, left_line_y)]
                            right_line_points = [self.transform_point(x, y) for x, y in zip(right_line_x, right_line_y)]
                            pygame.draw.lines(self.screen, self.COLOR_NOMAL_LINE, False, left_line_points, 3)
                            pygame.draw.lines(self.screen, self.COLOR_NOMAL_LINE, False, right_line_points, 3)
            if road["junction"] == None:
                # center line
                center_line = road["center_line"]
                xs = center_line["x"]
                ys = center_line["y"]
                color = self.COLOR_CENTER_LANE
                transformed_points = [self.transform_point(x, y) for x, y in zip(xs, ys)]
                self.draw_dashed_line(self.COLOR_CENTER_LANE, transformed_points)

    def screen_to_map(self, screen_x, screen_y):
        """ Convert screen coordinates to map coordinates. """
        map_x = (screen_x - self.offset_x) / self.scale
        map_y = -(screen_y - self.offset_y) / self.scale
        return map_x, map_y
    
    def run(self):
        dragging = False
        last_mouse_pos = None
        running = True
        while running:
            self.screen.fill((255, 255, 255))
            mouse_pos = pygame.mouse.get_pos()
            map_x, map_y = self.screen_to_map(*mouse_pos)

            for event in pygame.event.get():
                # Close application on quit event
                if event.type == pygame.QUIT:
                    running = False
                # Control map position and zoom with keyboard
                elif event.type == pygame.KEYDOWN:
                    print(f"Key pressed: {pygame.key.name(event.key)}")
                    if event.key == pygame.K_UP:
                        self.offset_y -= 10
                    elif event.key == pygame.K_DOWN:
                        self.offset_y += 10
                    elif event.key == pygame.K_LEFT:
                        self.offset_x -= 10
                    elif event.key == pygame.K_RIGHT:
                        self.offset_x += 10
                    elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                        self.scale *= 1.1
                    elif event.key == pygame.K_MINUS:
                        self.scale *= 0.9
                    elif event.key == pygame.K_0:
                        print("Resetting view")
                        self.offset_x = 150
                        self.offset_y = -280
                        self.scale = 4
                # Control panning and zooming with mouse
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 4:
                        self.scale *= 1.1
                    elif event.button == 5:
                        self.scale *= 0.9
                    elif event.button == 1:
                        dragging = True
                        last_mouse_pos = pygame.mouse.get_pos()
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        dragging = False
                elif event.type == pygame.MOUSEMOTION:
                    if dragging:
                        mouse_x, mouse_y = pygame.mouse.get_pos()
                        dx = mouse_x - last_mouse_pos[0]
                        dy = mouse_y - last_mouse_pos[1]
                        self.offset_x += dx
                        self.offset_y += dy
                        last_mouse_pos = (mouse_x, mouse_y)

            self.draw_map()

            font = pygame.font.Font(None, 30)
            text_surface = font.render(f"x: {map_x:.2f} y: {map_y:.2f}", True, (0, 0, 0))
            self.screen.blit(text_surface, (10, 10))

            pygame.display.flip()
            self.clock.tick(60)
        
        pygame.quit()