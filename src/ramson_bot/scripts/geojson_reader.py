import json
from dataclasses import dataclass
from typing import Optional, List, Dict
import enum
import os
from ament_index_python.packages import get_package_share_directory

class EdgeDirection(enum.Enum):
    LEFT = 'l'
    RIGHT = 'r'
    FORWARD = 'f'
    NONE = ''

@dataclass
class Node:
    id: int
    properties: dict

@dataclass
class Edge:
    id: int
    start_id: int
    end_id: int
    direction: EdgeDirection
    properties: dict

class Graph:
    def __init__(self, geojson_path: str):
        with open(geojson_path, encoding='utf-8') as f:
            data = json.load(f)
        features = data.get('features', [])
        
        self.nodes: Dict[int, Node] = {}
        self.edges: Dict[int, Edge] = {}
        self.adjacency: Dict[int, List[int]] = {}
        self.need_detection_nodes: List[int] = []
        
        for feat in features:
            props = feat.get('properties', {})
            feature_id = props.get('id')
            geom_type = feat.get('geometry', {}).get('type')
            
            if geom_type == 'Point' and feature_id is not None:
                self.nodes[feature_id] = Node(id=feature_id, properties=props)
                self.adjacency[feature_id] = []
                need_detection = props.get('need_detection', False)
                if need_detection:
                    self.need_detection_nodes.append(feature_id)
            elif geom_type == 'MultiLineString' and feature_id is not None:
                start_id = props.get('startid')
                end_id = props.get('endid')
                direction = EdgeDirection(props.get('direction', ''))
                edge = Edge(id=feature_id, start_id=start_id, end_id=end_id, direction=direction, properties=props)
                self.edges[feature_id] = edge
                if start_id in self.adjacency:
                    self.adjacency[start_id].append(feature_id)
                # if end_id in self.adjacency:
                #     self.adjacency[end_id].append(feature_id)

    def get_node(self, node_id: int) -> Optional[Node]:
        return self.nodes.get(node_id)

    def get_edges(self, node_id: int) -> List[Edge]:
        edge_ids = self.adjacency.get(node_id, [])
        return [self.edges[eid] for eid in edge_ids]

    def get_turn_edge(self, node_id: int, turn: EdgeDirection) -> Optional[Edge]:
        for edge in self.get_edges(node_id):
            if edge.direction == turn:
                return edge
        return None

if __name__ == "__main__":
    graph_path = os.path.join(get_package_share_directory('ramson_bot'), 'graphs', 'my_graph.geojson')
    graph = Graph(graph_path)

    print(f"Особые вершины: {[id for id in graph.need_detection_nodes]}")
    
    node = graph.get_node(8)
    print(f"Вершина 8: {node}")
    
    edges = graph.get_edges(8)
    print(f"Рёбра из узла 8: {[f"{e.id}: {e.direction.name}" for e in edges]}")
    
    right_edge = graph.get_turn_edge(8, EdgeDirection.RIGHT)
    if right_edge:
        print(f"Правый поворот из узла 8 — ребро {right_edge.id}")
    else:
        print("Правый поворот из узла 8 отсутствует.")
