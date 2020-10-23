%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Grid size X: 10
%Grid size Y: 7
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#program base.


%init
init(object(node, 31), value(at, (1, 4))).
init(object(node, 21), value(at, (1, 3))).
init(object(node, 11), value(at, (1, 2))).
init(object(node, 12), value(at, (2, 2))).
init(object(node, 14), value(at, (4, 2))).
init(object(node, 15), value(at, (5, 2))).
init(object(node, 16), value(at, (6, 2))).
init(object(node, 17), value(at, (7, 2))).
init(object(node, 27), value(at, (7, 3))).
init(object(node, 47), value(at, (7, 5))).
init(object(node, 45), value(at, (5, 5))).
init(object(node, 44), value(at, (4, 5))).
init(object(node, 43), value(at, (3, 5))).
init(object(node, 42), value(at, (2, 5))).
init(object(node, 41), value(at, (1, 5))).
init(object(node, 4), value(at, (4, 1))).
init(object(node, 48), value(at, (8, 5))).
init(object(node, 49), value(at, (9, 5))).
init(object(node, 22), value(at, (2, 3))).
init(object(node, 46), value(at, (6, 5))).
init(object(node, 32), value(at, (2, 4))).
init(object(node, 18), value(at, (8, 2))).
init(object(node, 28), value(at, (8, 3))).
init(object(node, 38), value(at, (8, 4))).
init(object(node, 57), value(at, (7, 6))).
init(object(node, 67), value(at, (7, 7))).
init(object(node, 50), value(at, (10, 5))).
init(object(highway, 31), value(at, (1, 4))).
init(object(highway, 21), value(at, (1, 3))).
init(object(highway, 11), value(at, (1, 2))).
init(object(highway, 12), value(at, (2, 2))).
init(object(highway, 14), value(at, (4, 2))).
init(object(highway, 41), value(at, (1, 5))).
init(object(highway, 42), value(at, (2, 5))).
init(object(highway, 43), value(at, (3, 5))).
init(object(highway, 44), value(at, (4, 5))).
init(object(highway, 45), value(at, (5, 5))).
init(object(highway, 46), value(at, (6, 5))).
init(object(highway, 47), value(at, (7, 5))).
init(object(highway, 17), value(at, (7, 2))).
init(object(highway, 16), value(at, (6, 2))).
init(object(highway, 15), value(at, (5, 2))).
init(object(highway, 48), value(at, (8, 5))).
init(object(highway, 49), value(at, (9, 5))).
init(object(highway, 32), value(at, (2, 4))).
init(object(highway, 38), value(at, (8, 4))).
init(object(highway, 28), value(at, (8, 3))).
init(object(highway, 18), value(at, (8, 2))).
init(object(highway, 67), value(at, (7, 7))).
init(object(highway, 50), value(at, (10, 5))).
init(object(pickingStation,1), value(at,(4,1))).

init(object(robot,1), value(at,(7,5))).
init(object(shelf,1), value(at,(2,3))).
init(object(product,1),value(on,(1,10))).
init(object(order,1),value(pickingStation,1)).
init(object(order,1),value(line,(1,1))).

%init(object(robot,2), value(at,(4,5))).
init(object(shelf,2), value(at,(7,3))).
init(object(product,2),value(on,(2,10))).
init(object(order,2),value(pickingStation,1)).
init(object(order,2),value(line,(2,2))).

%init(object(robot,3), value(at,(7,7))).
init(object(shelf,3), value(at,(7,6))).
init(object(product,3),value(on,(3,10))).
init(object(order,3),value(pickingStation,1)).
init(object(order,3),value(line,(3,3))).
