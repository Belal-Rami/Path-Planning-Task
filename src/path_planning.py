from __future__ import annotations

from typing import List

from src.models import CarPose, Cone, Path2D

import math


class PathPlanning:

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def sortConeByColor(self, color: int) -> list[Cone]:
        cones = []
        for cone in self.cones:
            if cone.color == color:
                cones.append(cone)

        return sorted(cones, key=lambda cone: math.hypot(cone.x - self.car_pose.x, cone.y - self.car_pose.y))
    
    def linearInterpolitation(self, path: Path2D, step: float) -> Path2D:
        
        new_path: Path2D = [path[0]]
        for i in range(len(path)-1):
            point1 = path[i]
            point2 = path[i+1]

            distance = math.hypot(point2[0]-point1[0], point2[1]-point1[1])

            num_steps = int(math.ceil(distance / step))

            for y in range(1, num_steps+1):
                dx= point1[0] + (y/num_steps) * (point2[0]-point1[0])
                dy = point1[1] + (y/num_steps) * (point2[1]-point1[1])
                new_path.append((dx, dy))

        return new_path

    def coneInflation(self, path: Path2D, min_distance: float):
        for i in range(len(path)):
            for cone in self.cones:
                dx = path[i][0]-cone.x
                dy = path[i][1]-cone.y
                distance = math.hypot(dx, dy)
                if  distance < min_distance:
                    path[i] = (cone.x + (dx/distance)*0.5, cone.y + (dy/distance)*0.5)
        return path


    def singleCone(self, point: tuple [float, float], dist: float, cone: Cone):
        dx = cone.x - point[0]
        dy = cone.y - point[1]

        distance = math.hypot(dx, dy)

        dx = dx/distance
        dy = dy/distance

        point = (cone.x + dy*dist, cone.y + dx*dist)

        if cone.color == 1:
            offset_x = dy * dist
            offset_y = -dx * dist
        else: 
            offset_x = -dy * dist
            offset_y = dx * dist

        return (cone.x + offset_x, cone.y + offset_y)

    def splitSides(self, blue_cones: list[Cone], yellow_cones: list[Cone]):
        if len(blue_cones) <= len(yellow_cones):
            return blue_cones, yellow_cones
        return yellow_cones, blue_cones

    def findNearestCone(self, cone: Cone, candidates: list[Cone]) -> Cone:
        nearest = candidates[0]
        nearest_distance = math.hypot(nearest.x - cone.x, nearest.y - cone.y)

        for candidate in candidates:
            distance = math.hypot(candidate.x - cone.x, candidate.y - cone.y)
            if distance < nearest_distance:
                nearest = candidate
                nearest_distance = distance

        return nearest

    def addPairMidpoints(self, short_side: list[Cone], long_side: list[Cone], path: Path2D) -> list[Cone]:
        leftovers = list(long_side)

        for cone in short_side:
            partner = self.findNearestCone(cone, leftovers)
            leftovers.remove(partner)

            mid_x = (cone.x + partner.x)/2
            mid_y = (cone.y + partner.y)/2
            path.append((mid_x, mid_y))

        return leftovers

    def addDetachedCones(self, leftovers: list[Cone], long_side: list[Cone], path: Path2D):
        for cone in leftovers:
            i = long_side.index(cone)

            if len(long_side) == 1:
                start = (self.car_pose.x, self.car_pose.y)
            else:
                k = min(i, len(long_side) - 2)
                dx = long_side[k+1].x - long_side[k].x
                dy = long_side[k+1].y - long_side[k].y
                start = (cone.x - dx, cone.y - dy)

            path.append(self.singleCone(start, 1.0, cone))

    def sortPathPoints(self, path: Path2D) -> Path2D:
        car_point = path[0]
        points = sorted(path[1:], key=lambda p: math.hypot(p[0] - car_point[0], p[1] - car_point[1]))

        return [car_point] + points
    def smoothPath(self, path: Path2D, passes: int) -> Path2D:
        for p in range(passes):
            new_path: Path2D = [path[0]]
            for i in range(1, len(path)-1):
                x = (path[i-1][0] + path[i][0] + path[i+1][0])/3
                y = (path[i-1][1] + path[i][1] + path[i+1][1])/3
                new_path.append((x, y))
            new_path.append(path[-1])
            path = new_path
        return path
    def generatePath(self) -> Path2D:

        blue_cones = self.sortConeByColor(1)
        yellow_cones = self.sortConeByColor(0)
        path1: Path2D = []
        path1.append((self.car_pose.x, self.car_pose.y))
        ahead_x = self.car_pose.x + math.cos(self.car_pose.yaw)
        ahead_y = self.car_pose.y + math.sin(self.car_pose.yaw)
        path1.append((ahead_x, ahead_y))

        short_side, long_side = self.splitSides(blue_cones, yellow_cones)
        leftovers = self.addPairMidpoints(short_side, long_side, path1)
        self.addDetachedCones(leftovers, long_side, path1)
        path1 = self.sortPathPoints(path1)

        ahead_x = self.car_pose.x + math.cos(self.car_pose.yaw)
        ahead_y = self.car_pose.y + math.sin(self.car_pose.yaw)
       
        path1 = self.linearInterpolitation(path1, 0.5)
        path1 = self.smoothPath(path1, 3)
        self.coneInflation(path1, 0.5)
        print(blue_cones)
        print(yellow_cones)
        print(path1)

        return path1