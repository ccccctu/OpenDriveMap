import unittest
from modules.map.map_manager import MapManager
from modules.map.visualizer import Visualizer


def main():
    map_file_path = "map_file/Town02.xodr"
    draw_data_path = "map_file/Town02.json"
    
    map_manager = MapManager(map_file_path)
    map_manager.calculate_lane_line_and_center_line()
    map_manager.export_map_data_for_visualize(draw_data_path)
    
    visualizer = Visualizer(draw_data_path)
    visualizer.run()

if __name__ == "__main__":
    main()