import { PriorityList } from "lua-priority-list";

export type Coordinate = {
    x: number;
    y: number;
};

export type Line = {
    nextX: number;
    nextY: number;
} & Coordinate;

type Collidable = {
    blocked: boolean;
    travelCost: number;
} & Coordinate;

type eCollidable = {
    pathCost?: number;
    heuristicToEnd?: number;
    previous?: eCollidable;
    totalWeight?: number;
    inOpenList?: boolean;
    inClosedList?: boolean;
} & Collidable;

type eCollidableList = eCollidable[];

type CollidableRow = Collidable[];

export type CollidableMap = eCollidableList[];

let straightDistance = 1;
let diagonalDistance = straightDistance * math.sqrt(2);

/**
 * Allows you to replace the heuristic distances
 * @param straight Cost of going straight
 * @param diagonal Cost of going diagonally
 */
export const configureHeuristicDistances = (straight: number, diagonal: number) => {
    straightDistance = straight;
    diagonalDistance = diagonal;
}

let heuristic = (node: Collidable, goal: Coordinate): number => {
    const dx = math.abs(node.x - goal.x);
    const dy = math.abs(node.y - goal.y);

    return straightDistance * (dx + dy) + (diagonalDistance - 2 * straightDistance) * math.min(dx, dy);
};

/**
 * Allows you to replace the default heuristic function if you got something exotic
 * @param newH the new heuristic function
 */
export const replaceHeuristicFunction = (newH: (node: Collidable, goal: Coordinate) => number) => {
    heuristic = newH;
}

// The list of navigable neighbors for a tile, remove math.sqrt(2) entries for only square movement etc.
// The path cost is the geometric cost of travelling from a tile to the left or diagonally for example
let neighbors = [
    { x: -1, y:  0, pathCost: 1 },
    { x:  1, y:  0, pathCost: 1 },
    { x:  0, y: -1, pathCost: 1 },
    { x:  0, y: +1, pathCost: 1 },
    { x: -1, y: -1, pathCost: math.sqrt(2), blockedBy: [0, 2] },
    { x: -1, y: +1, pathCost: math.sqrt(2), blockedBy: [0, 5] },
    { x:  1, y: -1, pathCost: math.sqrt(2), blockedBy: [2, 1] },
    { x:  1, y: +1, pathCost: math.sqrt(2), blockedBy: [3, 1] },
];

/*
       x
y  -1  0 +1
-1 [] [] []
0  []    []
+1 [] [] []

*/

interface Neighbor {x: number, y:number, pathCost: number, blockedBy?: number[]};

/**
 * Allows you to reconfigure the list of the pathable neighbors
 * @param n The list (see original source for how it should look)
 */
export const configurePathableNeighbors = (n: Neighbor[]) => {
    neighbors = n;
}

const checkTile = (
    target: eCollidable | undefined,
    parent: eCollidable,
    tileTravelCost: number,
    usedList: LuaTable<Collidable, eCollidable>,
    openList: PriorityList<eCollidable>,
    end: Coordinate
): void => {
    if (!target) {
        return;
    }

    if (!target.blocked) {
        const tcost = tileTravelCost;
        const cost = target.travelCost * tcost;
        const parentCost = parent.pathCost || 0;
        const pathCost = parentCost + cost;

        let inOpenList = openList.isPresent(target);
        let inClosedList = usedList.has(target);

        if (inOpenList) {
            if (target && target.pathCost && pathCost < target.pathCost) {
                openList.remove(target);
                inOpenList = false;
            }
        }
        if (inClosedList) {
            if (target.pathCost && target.pathCost > pathCost) {
                usedList.delete(target);
                inClosedList = false;
            }
        }
        if (!inOpenList && !inClosedList) {
            const hCost = heuristic(target, end);
            target.pathCost = pathCost;
            target.heuristicToEnd = hCost;
            target.previous = parent;
            target.totalWeight = pathCost + hCost;
            openList.put(pathCost + hCost, target);
            target.inOpenList = true;

            if (target.previous.previous === target) {
                error("loop detected" + target.x + target.y + ": " + target.previous.x + target.previous.y);
            }
        }
    }
};

const parseNeighbors = (
    map: CollidableMap,
    target: eCollidable,
    usedList: LuaTable<Collidable, eCollidable>,
    openList: PriorityList<eCollidable>,
    end: Coordinate
): typeof target | undefined => {
    if (target.x === end.x && target.y === end.y) {
        return target;
    }

    target.inClosedList = true;
    target.inOpenList = false;

    if (!usedList.has(map[target.x][target.y])) {
        usedList.set(map[target.x][target.y], target);
    }

    for (const i of $range(0, neighbors.length - 1)) {
        if (map[target.x + neighbors[i].x]) {
            const neighbor = map[target.x + neighbors[i].x][target.y + neighbors[i].y];
            if (!(neighbor === target.previous)) {
                if (neighbors[i].blockedBy){
                    const b = neighbors[i].blockedBy;
                    if (b){
                        const bi1 = b[0];
                        const bi2 = b[1];

                        const n2 = map[target.x + neighbors[bi1].x] 
                            && map[target.x + neighbors[bi1].x][target.y + neighbors[bi1].y]
                            && map[target.x + neighbors[bi1].x][target.y + neighbors[bi1].y].blocked;

                        const n3 = map[target.x + neighbors[bi2].x] 
                            && map[target.x + neighbors[bi2].x][target.y + neighbors[bi2].y]
                            && map[target.x + neighbors[bi2].x][target.y + neighbors[bi2].y].blocked;

                        if (!(n2&&n3)){
                            checkTile(neighbor, target, neighbors[i].pathCost, usedList, openList, end);
                        }
                    }
                } else {
                    checkTile(neighbor, target, neighbors[i].pathCost, usedList, openList, end);
                }
            }
        }
    }
};

/**
 * The pathing function, for performance reasons I recommend that your basic travel cost is just slightly below 1, like 0.95
 * @param map The map that will be pathed through
 * @param start Coordinates of the start
 * @param end Coordinates of the end
 * @returns Your final path, a series of lines, or series of coordinates, if you prefer
 */
export const astar = (map: CollidableMap, start: Coordinate, end: Coordinate): Line[] | undefined => {
    let finpath: eCollidable | undefined;
    const path: Coordinate[] = [];
    const usedList = new LuaTable<Collidable, eCollidable>();
    const h = heuristic(map[start.x][start.y], map[end.x][end.y]);
    const startNode = map[start.x][start.y];
    const tcost = startNode.travelCost;

    const openList = new PriorityList<eCollidable>();

    openList.put(
        h + tcost,
        {
            x: start.x,
            y: start.y,
            pathCost: tcost,
            travelCost: tcost,
            heuristicToEnd: h,
            totalWeight: h + tcost,
            blocked: startNode.blocked,
        },
    );

    while (openList.list.length > 0) {
        const res = parseNeighbors(map, openList.take(), usedList, openList, end);
        if (res) {
            finpath = res;
            break;
        }
    }

    // Extracting the inverse path
    let stopped = false;
    while (!stopped) {
        if (finpath) {
            path[path.length] = {
                x: finpath?.x,
                y: finpath?.y,
            };
            finpath = finpath.previous;
        } else {
            stopped = true;
        }
    }

    // Reversing the path
    const revpath: Line[] = [];
    if (path.length > 0) {
        for (const i of $range(0, path.length - 1)) {
            const nextX = path[i - 1] ? path[i - 1].x : path[i].x;
            const nextY = path[i - 1] ? path[i - 1].y : path[i].y;
            revpath[revpath.length] = {
                ...path[i],
                nextX: nextX,
                nextY: nextY,
            };
        }
    }

    openList.free();

    return revpath;
};
