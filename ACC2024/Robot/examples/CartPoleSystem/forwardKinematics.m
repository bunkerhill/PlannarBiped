clear
clc
addpath("../../")

%% World Frame Base
base = CoordinateFrame2D('Name', 'base')

%% Cart Frame: x axis transition
p = Point2D('X', sym('x'))
cart = CoordinateFrame2D('Name', 'cart', 'ParentFrame', base, 'Displacement', p)

%% Pole Frame: z (out of plane) axis rotation
r = SO2(sym('theta'))
pole = CoordinateFrame2D('Name', 'pole', 'ParentFrame', cart, 'Rotation', r)

%% Pole's Top End
poleEnd = CoordinateFrame2D('Name', 'poleEnd', 'ParentFrame', pole, 'Displacement', Point2D('Y', sym('l')))

forward = poleEnd.HomogeneousFromBase
endPoint = forward(1:2,3)
