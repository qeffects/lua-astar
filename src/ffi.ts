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

export type FFIStruct = {
    pathCost: number;
    heuristicToEnd: number;
    totalWeight: number;
    inOpenList: boolean;
    inClosedList: boolean;
} & Collidable;

type LuaNode = {
    previous?: LuaNode;
    ffiIndex: number;
}

type eCollidableList = FFIStruct[];

export type FFIMap = eCollidableList;

export type LuaMap<T = unknown> = LuaNode[] & {
    heuristic: (node: T, node2: T) => number;
    getNeighbors: (node: T) => LuaNode[];
    twoNodeDist: (node: T, node2: T) => number;
    getFFIindex: (node: T) => number;
    isEnd: (node: T) => boolean;
};

let straightDistance = 1;
let diagonalDistance = straightDistance * math.sqrt(2);

let heuristic = (node: Collidable, goal: Coordinate): number => {
    const dx = math.abs(node.x - goal.x);
    const dy = math.abs(node.y - goal.y);

    return straightDistance * (dx + dy) + (diagonalDistance - 2 * straightDistance) * math.min(dx, dy);
};

const checkTile = (
    thisNode: LuaNode | undefined,
    parentNode: LuaNode,
    usedList: LuaTable<LuaNode, LuaNode>,
    openList: PriorityList<LuaNode>,
    luaMap: LuaMap,
    ffiMap: FFIMap,
    end: LuaNode
): void => {
    if (!thisNode) {
        return;
    }

    if (thisNode.previous === parentNode){
        return;
    }
    
    const ffiIndex = thisNode.ffiIndex;

    if (!ffiMap[ffiIndex].blocked) {
        const tcost = luaMap.twoNodeDist(thisNode, parentNode);
        const cost = ffiMap[ffiIndex].travelCost * tcost;

        const parentCost = ffiMap[parentNode.ffiIndex].pathCost || 0;
        const pathCost = parentCost + cost;

        let inOpenList = openList.isPresent(thisNode);
        let inClosedList = usedList.has(thisNode);

        if (inOpenList) {
            if (thisNode && ffiMap[ffiIndex].pathCost && pathCost < ffiMap[ffiIndex].pathCost) {
                openList.remove(thisNode);
                inOpenList = false;
            }
        }
        if (inClosedList) {
            if (ffiMap[ffiIndex].pathCost && ffiMap[ffiIndex].pathCost > pathCost) {
                usedList.delete(thisNode);
                inClosedList = false;
            }
        }
        if (!inOpenList && !inClosedList) {
            const hCost = luaMap.heuristic(thisNode, end);

            ffiMap[ffiIndex].pathCost = pathCost;
            ffiMap[ffiIndex].heuristicToEnd = hCost;
            ffiMap[ffiIndex].totalWeight = pathCost + hCost;
            ffiMap[ffiIndex].inOpenList = true;
            
            thisNode.previous = parentNode;

            openList.put(pathCost + hCost, thisNode);

            if (thisNode.previous.previous === thisNode) {
                error("loop detected");
            }
        }
    }
};

const parseNeighbors = (
    ffiMap: FFIMap,
    luaMap: LuaMap,
    thisNode: LuaNode,
    usedList: LuaTable<LuaNode, LuaNode>,
    openList: PriorityList<LuaNode>,
    end: LuaNode
): typeof thisNode | undefined => {
    if (thisNode == end){
        return thisNode;
    }

    ffiMap[thisNode.ffiIndex].inClosedList = true;
    ffiMap[thisNode.ffiIndex].inOpenList = false;

    if (!usedList.has(thisNode)) {
        usedList.set(thisNode, thisNode);
    }
    
    const neighbors = luaMap.getNeighbors(thisNode);

    for (const neigh of neighbors){
        checkTile(neigh, thisNode, usedList, openList, luaMap, ffiMap, end);
    }
};

/**
 * The pathing function, for performance reasons I recommend that your basic travel cost is just slightly below 1, like 0.95
 * @param map The map that will be pathed through
 * @param start Coordinates of the start
 * @param end Coordinates of the end
 * @returns Your final path, a series of lines, or series of coordinates, if you prefer
 */
export const astar = <T>(ffiMap: FFIMap, luaMap: LuaMap, start: LuaNode, end: LuaNode): LuaNode[] | undefined => {
    let finpath: LuaNode | undefined;
    let steps: LuaNode[] = [];

    const usedList = new LuaTable<LuaNode, LuaNode>();
    const openList = new PriorityList<LuaNode>();

    const h = luaMap.heuristic(start, end);
    const ffiIndex = start.ffiIndex;

    openList.put(
        h + ffiMap[ffiIndex].travelCost,
        {
            ffiIndex: ffiIndex,
        },
    );

    while (openList.list.length > 0) {
        const res = parseNeighbors(ffiMap, luaMap, openList.take(), usedList, openList, end);
        if (res) {
            finpath = res;
            break;
        }
    }

    if (finpath) {
        let finished = false
        let node = finpath;

        while (!finished){
            if (!node.previous){
                break;
            }
            node = node.previous;
            steps.push(node);
        }
    }

    let revpath:LuaNode[]|undefined = undefined;
    if (steps.length>0){
        revpath = [];
        for (const node of steps){
            revpath.push(node);
        }
    }

    openList.free();

    return revpath;
};
