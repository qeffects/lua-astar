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

const straightDistance = 1;
const diagonalDistance = straightDistance * math.sqrt(2);

const heuristic = (node: Collidable, goal: Coordinate): number => {
    const dx = math.abs(node.x - goal.x);
    const dy = math.abs(node.y - goal.y);

    return straightDistance * (dx + dy) + (diagonalDistance - 2 * straightDistance) * math.min(dx, dy);
};

const checkTile = (
    target: eCollidable | undefined,
    parent: eCollidable,
    diag: boolean,
    usedList: LuaTable<Collidable, eCollidable>,
    openList: PriorityList<eCollidable>,
    end: Coordinate
): void => {
    if (!target) {
        return;
    }

    if (!target.blocked) {
        const tcost = diag ? diagonalDistance : straightDistance;
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
            openList.put(pathCost + hCost, target, hCost);
            target.inOpenList = true;

            if (target.previous.previous === target) {
                error("loop detected" + target.x + target.y + ": " + target.previous.x + target.previous.y);
            }
        }
    }
};

const neighbors = [
    { x: -1, y: 0, diag: false },
    { x: -1, y: -1, diag: true },
    { x: -1, y: +1, diag: true },
    { x: 0, y: -1, diag: false },
    { x: 0, y: +1, diag: false },
    { x: 1, y: 0, diag: false },
    { x: 1, y: -1, diag: true },
    { x: 1, y: +1, diag: true },
];

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
                checkTile(neighbor, target, neighbors[i].diag, usedList, openList, end);
            }
        }
    }
};

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
        h
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

    return revpath;
};
