#!/usr/bin/env python

import argparse, numpy, openravepy, time

from HerbRobot import HerbRobot
from SimpleRobot import SimpleRobot
from HerbEnvironment import HerbEnvironment
from SimpleEnvironment import SimpleEnvironment

from AStarPlanner import AStarPlanner
from DepthFirstPlanner import DepthFirstPlanner
from BreadthFirstPlanner import BreadthFirstPlanner
from HeuristicRRTPlanner import HeuristicRRTPlanner
from RRTPlanner import RRTPlanner
import IPython

def main(robot, planning_env, planner):
    print planner.__class__.__name__
    raw_input('Press any key to begin planning')

    start_config = numpy.array(robot.GetCurrentConfiguration())
    if robot.name == 'herb':
        goal_config = numpy.array([ 4.3, -1.76, 0.00, 1.96, -1.15, 0.87, -1.43] )
    else:
        goal_config = numpy.array([3.0, 0.0])

    plan_start = time.time()
    with robot.robot.GetEnv():
        plan = planner.Plan(start_config, goal_config)
    plan_time = time.time() - plan_start

    f = open('hrrt_results/results_7d_hrrt.txt', 'a')
    f.write("Planner = %s \n\n" % planner.__class__)

    if planner.__class__.__name__ == "RRTPlanner" or planner.__class__.__name__ == "HeuristicRRTPlanner":
        plan_length = planning_env.ComputePathSliceLength(plan, 0, len(plan)-1)

        #write out the number of nodes expanded
        f.write("plan time = %f \n" % plan_time)
        f.write("path length = %f \n" % plan_length)
        f.write("Nodes Expanded = %d \n" % len(planner.tree.vertices))
        shorter_path = planning_env.ShortenPath(plan)
        shorter_len = planning_env.ComputePathSliceLength(shorter_path, 0, len(shorter_path)-1)
        f.write("shortened path length = %f \n" % shorter_len)
    else:
        plan_length = planning_env.discrete_env.resolution*len(plan)
        f.write("plan time = %f \n" % plan_time)
        f.write("path length = %f \n" % plan_length)
        f.write("Resolution = %f \n" % planning_env.discrete_env.resolution)


    f.close()
    traj = robot.ConvertPlanToTrajectory(plan)

    raw_input('Press any key to execute trajectory')
    robot.ExecuteTrajectory(traj)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='script for testing planners')

    parser.add_argument('-r', '--robot', type=str, default='simple',
                        help='The robot to load (herb or simple)')
    parser.add_argument('-p', '--planner', type=str, default='astar',
                        help='The planner to run (astar, bfs, dfs or hrrt)')
    parser.add_argument('-v', '--visualize', action='store_true',
                        help='Enable visualization of tree growth (only applicable for simple robot)')
    parser.add_argument('--resolution', type=float, default=0.1,
                        help='Set the resolution of the grid (default: 0.1)')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Enable debug logging')
    parser.add_argument('-m', '--manip', type=str, default='right',
                        help='The manipulator to plan with (right or left) - only applicable if robot is of type herb')
    args = parser.parse_args()

    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()

    if args.debug:
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

    env = openravepy.Environment()
    env.SetViewer('qtcoin')
    env.GetViewer().SetName('Homework 3 Viewer')

    # First setup the environment and the robot
    visualize = args.visualize
    if args.robot == 'herb':
        print("HERBING IT UP")
        robot = HerbRobot(env, args.manip)
        planning_env = HerbEnvironment(robot, args.resolution)
        visualize = False
    elif args.robot == 'simple':
        print("It's a simple kinda plan")
        robot = SimpleRobot(env)
        planning_env = SimpleEnvironment(robot, args.resolution)
    else:
        print 'Unknown robot option: %s' % args.robot
        exit(0)

    # Next setup the planner
    if args.planner == 'astar':
        print("A*")
        planner = AStarPlanner(planning_env, visualize)
    elif args.planner == 'bfs':
        print("BFS")
        planner = BreadthFirstPlanner(planning_env, visualize)
    elif args.planner == 'dfs':
        print("DFS")
        planner = DepthFirstPlanner(planning_env, visualize)
    elif args.planner == 'hrrt':
        print("hrrt")
        planner = HeuristicRRTPlanner(planning_env, visualize)
    elif args.planner == 'rrt':
        print("rrt")
        planner = RRTPlanner(planning_env, visualize)
    else:
        print 'Unknown planner option: %s' % args.planner
        exit(0)

    main(robot, planning_env, planner)

    # import IPython
    # IPython.embed()

