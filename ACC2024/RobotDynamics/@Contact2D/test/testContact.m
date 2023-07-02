clear
clc

%% World Frame Base
base = CoordinateFrame2D('Name', 'base');

x = sym('x');
y = sym('y');
theta = sym('theta');

%% First link
link1 = CoordinateFrame2D('Name', 'link1', 'ParentFrame', base, 'Displacement', Point2D('X', x,'Y', y), 'Rotation', SO2(theta));

contactPoint = CoordinateFrame2D('Name', 'foot', 'ParentFrame', link1, 'Displacement', Point2D('Y', -sym('l')/2));

footContactPoint = Contact2D('Name', 'foot', 'CoordinateFrame', contactPoint)

footContactPoint.getContactPoint()

generalCoordinates = [x, y, theta];

footContactPoint.getContactJacobian(generalCoordinates)