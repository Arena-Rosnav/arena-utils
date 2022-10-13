import rospy
from nav_msgs.srv import GetMap
from map_distance_server.srv import GetDistanceMap, GetDistanceMapResponse
import numpy as np

def print_map(map):
    for row in map:
        print(list(row))

class MapDistanceServer:
    def __init__(self):
        rospy.wait_for_service("/static_map")

        map_service = rospy.ServiceProxy("/static_map", GetMap)
        self.map = map_service().map

        self.new_map_data = list(self._get_map_with_distances())

        self.distance_map_srv = rospy.Service(
            "/distance_map",
            GetDistanceMap, 
            self._distance_map_srv_handler
        )

    def _distance_map_srv_handler(self, _):
        msg = GetDistanceMapResponse()

        msg.header = self.map.header
        msg.info = self.map.info

        msg.data = self.new_map_data

        return msg

    def _get_map_with_distances(self):
        width_in_cell, height_in_cell = self.map.info.width, self.map.info.height

        map_2d = np.reshape(self.map.data, (height_in_cell, width_in_cell))

        free_space_indices = np.where(map_2d == 0)
        free_space_coordinates = np.array(free_space_indices).transpose()

        coordinates_with_length = np.full(len(self.map.data), -1) ## Hier wird die Länge gespeichert
        coordinates_length_dict = {} # Für schnellen Zugriff dict mit key = Länge und value = Array[(x, y)]

        # Loop over all free spaces and all neighbors and set distance of current
        # cell to 0 if neighbors have no distance (are obstacles) or to
        # nearest distance + 1 (cell is one more step away from obstacle than
        # neighbor) 
        for x, y in free_space_coordinates:
            dist = float("inf")

            for j in range(-1, 2):
                for i in range(-1, 2):

                    if (i == 0 and j == 0) or x + j < 0 or y + i < 0:
                        continue

                    try:
                        val = map_2d[x + j, y + i]
                    except:
                        continue

                    if val != 0:
                        dist = 0
                        continue

                    index = self._get_index(x + j, y + i)

                    if coordinates_with_length[index] >= 0:
                        dist = min(coordinates_with_length[index] + 1, dist)

            coordinates_length_dict.setdefault(dist, []).append((x, y))
            coordinates_with_length[self._get_index(x, y)] = dist

        min_key, max_key = min(coordinates_length_dict.keys()), max(coordinates_length_dict.keys())

        # Loop again over all cells from lowest to highest distance and update
        # all direct neighbors -> Set all distance to max current_dist + 1
        for key in range(min_key, max_key + 1):

            if not coordinates_length_dict.get(key):
                continue

            for x, y in coordinates_length_dict[key]:

                for j in range(-1, 2):
                    for i in range(-1, 2):
                        if (i == 0 and j == 0) or x + j < 0 or y + i < 0:
                            continue

                        try:
                            val = map_2d[x + j, y + i]
                        except:
                            ## Out of bounds
                            continue
                        
                        index = self._get_index(x + j, y + i)

                        if coordinates_with_length[index] > key + 1:
                            coordinates_with_length[index] = key + 1
                            coordinates_length_dict.setdefault(key + 1, []).append((x + j, y + i))

        # print(list(coordinates_with_length))

        return coordinates_with_length
        # return np.reshape(coordinates_with_length, (height_in_cell, width_in_cell))

    def _get_index(self, x, y):
        return x * self.map.info.width + y

if __name__ == "__main__":
    rospy.init_node("map_distance_server")

    distance_server = MapDistanceServer()

    while not rospy.is_shutdown():
        rospy.spin()

    # map = map_service().map

    # width_in_cell, height_in_cell = map.info.width, map.info.height

    # map_2d = np.reshape(map.data, (height_in_cell, width_in_cell))

    # print_map(map_2d)

    # free_space_indices = np.where(map_2d == 0)

    # free_space_coordinates = np.array(free_space_indices).transpose()

    # # For every cell we calculate the distance to the nearest obstacle
    # # Therefore calculate the distance to every blocked cell and take the lowest
    # # distance. This is roughly n^2, but this is only done once so it is OK

    # coordinates_with_length = np.full(len(map.data), -1) ## Hier wird die Länge gespeichert
    # coordinates_length_dict = {} # Für schnellen Zugriff dict mit key = Länge und value = Array[(x, y)]

    # def get_index(x, y):
    #     return x * map.info.width + y

    # for x, y in free_space_coordinates:
    #     dist = float("inf")

    #     for j in range(-1, 2):
    #         for i in range(-1, 2):

    #             if (i == 0 and j == 0) or x + j < 0 or y + i < 0:
    #                 continue

    #             try:
    #                 val = map_2d[x + j, y + i]
    #             except:
    #                 continue

    #             if val != 0:
    #                 dist = 0
    #                 continue

    #             index = get_index(x + j, y + i)

    #             if coordinates_with_length[index] >= 0:
    #                 dist = min(coordinates_with_length[index] + 1, dist)

    #     coordinates_length_dict.setdefault(dist, []).append((x, y))
    #     coordinates_with_length[get_index(x, y)] = dist

    # print("DISTANCES")

    # min_key, max_key = min(coordinates_length_dict.keys()), max(coordinates_length_dict.keys())

    # for key in range(min_key, max_key + 1):

    #     if not coordinates_length_dict.get(key):
    #         continue

    #     for x, y in coordinates_length_dict[key]:

    #         for j in range(-1, 2):
    #             for i in range(-1, 2):
    #                 if (i == 0 and j == 0) or x + j < 0 or y + i < 0:
    #                     continue

    #                 try:
    #                     val = map_2d[x + j, y + i]
    #                 except:
    #                     ## Out of bounds
    #                     continue
                    
    #                 index = get_index(x + j, y + i)

    #                 if coordinates_with_length[index] > key + 1:
    #                     coordinates_with_length[index] = key + 1
    #                     coordinates_length_dict.setdefault(key + 1, []).append((x + j, y + i))
                        
    # print("finished", map.info, min(coordinates_with_length), max(coordinates_with_length))

