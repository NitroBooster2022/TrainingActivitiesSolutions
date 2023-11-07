#!/usr/bin/env python3

import networkx as nx
import numpy as np
import cv2
import os
import json

class track_map():

    def __init__(self):
        self.map_graph=nx.read_graphml(os.path.dirname(os.path.realpath(__file__))+'/templates/Competition_track.graphml')

    def draw_map_graphml(self):
        # draw the path (graphml)
        img_map=cv2.imread(os.path.dirname(os.path.realpath(__file__))+'/templates/map.png')
        for n in self.map_graph.nodes():
            cv2.circle(img_map, (int(float(self.map_graph.nodes[n]['x'])/15*img_map.shape[0]),int(float(self.map_graph.nodes[n]['y'])/15*img_map.shape[1])), radius=15, color=(0,255,0), thickness=-1)
        for e in self.map_graph.edges():
            source = e[0]
            dest = e[1]
            img_map = cv2.arrowedLine(img_map, (int(self.map_graph.nodes[source]['x']/15*img_map.shape[0]),int(self.map_graph.nodes[source]['y']/15*img_map.shape[1])),
                    ((int(self.map_graph.nodes[dest]['x']/15*img_map.shape[0]),int(self.map_graph.nodes[dest]['y']/15*img_map.shape[1]))), color=(255,0,255), thickness=5)
        windowName = 'track'
        cv2.namedWindow(windowName,cv2.WINDOW_NORMAL)
        cv2.resizeWindow(windowName,700,700)
        cv2.imshow(windowName, img_map)
        key = cv2.waitKey(0)

    def plan_path_graphml(self, location, destination):
        # Calculate the shortest path
        img_map=cv2.imread(os.path.dirname(os.path.realpath(__file__))+'/templates/map.png')
        shortest_path = nx.shortest_path(self.map_graph, source=location, target=destination)
        print(shortest_path)

        # Draw the shortest path on the map
        for i in range(len(shortest_path) - 1):
            source_node = shortest_path[i]
            dest_node = shortest_path[i + 1]
            source_coords = (int(float(self.map_graph.nodes[source_node]['x'])/15*img_map.shape[0]), int(float(self.map_graph.nodes[source_node]['y'])/15*img_map.shape[1]))
            dest_coords = (int(float(self.map_graph.nodes[dest_node]['x'])/15*img_map.shape[0]), int(float(self.map_graph.nodes[dest_node]['y'])/15*img_map.shape[1]))
            img_map = cv2.arrowedLine(img_map, source_coords, dest_coords, color=(255, 0, 255), thickness=5)
            cv2.circle(img_map, source_coords, radius=15, color=(0,255,0), thickness=-1)

        windowName = 'track'
        cv2.namedWindow(windowName, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(windowName, 700, 700)
        cv2.imshow(windowName, img_map)
        key = cv2.waitKey(0)

if __name__ == '__main__':
    node = track_map()
    node.draw_map_graphml()
    node.plan_path_graphml('1','100')