%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Grid size X: 7
%Grid size Y: 5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Die Roboter haben hier keinen Platz zum navigieren


#program base.


%init
init(object(node, 3), value(at, (3, 1))).
init(object(node, 5), value(at, (5, 1))).
init(object(node, 8), value(at, (1, 2))).
init(object(node, 9), value(at, (2, 2))).
init(object(node, 10), value(at, (3, 2))).
init(object(node, 11), value(at, (4, 2))).
init(object(node, 12), value(at, (5, 2))).
init(object(node, 13), value(at, (6, 2))).
init(object(node, 14), value(at, (7, 2))).
init(object(node, 15), value(at, (1, 3))).
init(object(node, 16), value(at, (2, 3))).
init(object(node, 17), value(at, (3, 3))).
init(object(node, 18), value(at, (4, 3))).
init(object(node, 19), value(at, (5, 3))).
init(object(node, 20), value(at, (6, 3))).
init(object(node, 21), value(at, (7, 3))).
init(object(node, 22), value(at, (1, 4))).
init(object(node, 23), value(at, (2, 4))).
init(object(node, 24), value(at, (3, 4))).
init(object(node, 25), value(at, (4, 4))).
init(object(node, 26), value(at, (5, 4))).
init(object(node, 27), value(at, (6, 4))).
init(object(node, 28), value(at, (7, 4))).
init(object(node, 29), value(at, (1, 5))).
init(object(node, 30), value(at, (2, 5))).
init(object(node, 31), value(at, (3, 5))).
init(object(node, 32), value(at, (4, 5))).
init(object(node, 33), value(at, (5, 5))).
init(object(node, 34), value(at, (6, 5))).
init(object(node, 35), value(at, (7, 5))).
init(object(highway, 8), value(at, (1, 2))).
init(object(highway, 9), value(at, (2, 2))).
init(object(highway, 10), value(at, (3, 2))).
init(object(highway, 11), value(at, (4, 2))).
init(object(highway, 12), value(at, (5, 2))).
init(object(highway, 13), value(at, (6, 2))).
init(object(highway, 14), value(at, (7, 2))).
init(object(highway, 15), value(at, (1, 3))).
init(object(highway, 22), value(at, (1, 4))).
init(object(highway, 29), value(at, (1, 5))).
init(object(highway, 30), value(at, (2, 5))).
init(object(highway, 31), value(at, (3, 5))).
init(object(highway, 32), value(at, (4, 5))).
init(object(highway, 33), value(at, (5, 5))).
init(object(highway, 34), value(at, (6, 5))).
init(object(highway, 35), value(at, (7, 5))).
init(object(highway, 21), value(at, (7, 3))).
init(object(highway, 28), value(at, (7, 4))).
init(object(pickingStation,1), value(at,(3,1))).
init(object(pickingStation,2), value(at,(5,1))).
init(object(robot,1), value(at,(2,5))).
init(object(robot,3), value(at,(4,5))).
init(object(robot,2), value(at,(3,5))).
init(object(robot,5), value(at,(6,5))).
init(object(robot,4), value(at,(5,5))).
init(object(shelf,10), value(at,(4,3))).
init(object(shelf,1), value(at,(3,3))).
init(object(product,3),value(on,(1,8))).
init(object(product,4),value(on,(1,1))).
init(object(product,5),value(on,(1,10))).
init(object(shelf,3), value(at,(6,3))).
init(object(product,2),value(on,(3,4))).
init(object(product,4),value(on,(3,1))).
init(object(shelf,2), value(at,(5,3))).
init(object(product,2),value(on,(2,7))).
init(object(product,4),value(on,(2,1))).
init(object(product,5),value(on,(2,10))).
init(object(shelf,5), value(at,(2,3))).
init(object(shelf,4), value(at,(2,4))).
init(object(shelf,7), value(at,(4,4))).
init(object(shelf,6), value(at,(3,4))).
init(object(product,2),value(on,(6,7))).
init(object(product,5),value(on,(6,7))).
init(object(shelf,9), value(at,(6,4))).
init(object(product,1),value(on,(9,4))).
init(object(shelf,8), value(at,(5,4))).
init(object(order,1),value(pickingStation,1)).
init(object(order,1),value(line,(2,11))).
init(object(order,3),value(pickingStation,2)).
init(object(order,3),value(line,(1,2))).
init(object(order,2),value(pickingStation,2)).
init(object(order,2),value(line,(4,1))).
init(object(order,5),value(pickingStation,2)).
init(object(order,5),value(line,(3,2))).
init(object(order,4),value(pickingStation,1)).
init(object(order,4),value(line,(5,2))).
