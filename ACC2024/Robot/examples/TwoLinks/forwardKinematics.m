clear
clc

addpath("../../")
base = CoordinateFrame2D('Name', 'base')
%% Link 1: rotation theta1
link1 = CoordinateFrame2D('Name', 'link1', 'ParentFrame', base, 'Rotation', SO2(sym('theta1')))

%% Link 1 end effector
link1End = CoordinateFrame2D('Name', 'link1End', 'ParentFrame', link1, 'Displacement', Point2D('X', sym('l1')))

%% Link 2: rotation theta2
link2 = CoordinateFrame2D('Name', 'link2', 'ParentFrame', link1End, 'Rotation', SO2(sym('theta2')))

%% Link 2: end effector
link2End = CoordinateFrame2D('Name', 'link2End', 'ParentFrame', link2, 'Displacement', Point2D('X', sym('l2')))

forward = link2End.HomogeneousFromBase
endPoint = simplify(forward(1:2,3))
