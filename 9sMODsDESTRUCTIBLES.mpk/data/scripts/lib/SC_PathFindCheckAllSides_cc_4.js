using(console);

include(SC_PathFindUtilities_cc_4.js);

var SC_PathFindCheckAllSides_cc_4 =
{
    checkAllSidesGridIndex: function (node, gridWorldSize, cData, initialPos)
    {
        var neighboors = [];
        var extraTiles = [];
        for (var x = -1; x <= 1; x++)
        {
            for (var y = -1; y <= 1; y++)
            {
                if (x == 0 && y == 0)
                {
                    continue;
                }

                var xpos = Math.round(node.worldPosition.x + x);
                var ypos = Math.round(node.worldPosition.y + y);

                var pos = { x: xpos, y: ypos };

                var gridData = SC_PathFindUtilities_cc_4.getNewGridIndex(initialPos, pos, gridWorldSize, cData.objt.sid);

                var indexOfGrid = gridData.index;

                var xpos = Math.round(node.worldPosition.x + x);
                var ypos = Math.round(node.worldPosition.y + y);

                var pos = { x: xpos, y: ypos };

                if (xpos >= node.gridPos.x - gridWorldSize.xL && xpos <= node.gridPos.x + gridWorldSize.xR &&
                    ypos >= node.gridPos.y - gridWorldSize.yB && ypos <= node.gridPos.y + gridWorldSize.yT)
                {
                    var gridTileX = node.gridTileX + x;
                    var gridTileY = node.gridTileY + y;

                    if (gridTileX < 0)
                    {
                        gridTileX = (gridWorldSize.xR) + (gridTileX * -1);
                    }

                    if (gridTileY < 0)
                    {
                        gridTileY = (gridWorldSize.yT) + (gridTileY * -1);
                    }
                    var index = ((gridTileX) * (gridWorldSize.xL + gridWorldSize.xR + 1)) + (gridTileY);


                    neighboors.push({ swtc: 0, node: cData.log[node.gridIndex].grid[index], sgp: null, iot: null, iog: null});
                }
                else
                {             
                    var gridData = SC_PathFindUtilities_cc_4.getNewGridIndex(initialPos, pos, gridWorldSize, cData.objt.sid);

                    var indexOfGrid = gridData.index;
     
                    var diffX = Math.round(Math.abs(Math.abs(pos.x) - Math.abs(gridData.gridData.x)));
                    var diffY = Math.round(Math.abs(Math.abs(pos.y) - Math.abs(gridData.gridData.y)));

                    var starterGriderPos = { x: gridData.gridData.x, y: gridData.gridData.y };

                    //console.PrintError("iog: " + indexOfGrid);

                    //var idObj = generator.AddSpecialObject(cData.objt.sid, gridData.gridData.x, gridData.gridData.y, "waypoint_02", 0);
                    //
                    //if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                    //    var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                    //    crates.push(idObj);
                    //    storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    //}
                    //else {
                    //    var crates = [];
                    //    crates.push(idObj);
                    //    storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    //}
                   
                    if (pos.x < starterGriderPos.x) {
                        diffX = (gridWorldSize.xR) + diffX;
                    }
                    else if (pos.x > starterGriderPos.x) {
                        //console.PrintError("testing");
                        diffX = diffX;
                    }
                    else {
                        diffX = 0;
                    }

                    if (pos.y < starterGriderPos.y)
                    {
                        diffY = (gridWorldSize.yT) + diffY;
                    }
                    else if (pos.y > starterGriderPos.y)
                    {
                        diffY = diffY;
                    }
                    else {
                        diffY = 0;
                    }

                    var indexer = diffX * (gridWorldSize.xL + gridWorldSize.xR + 1) + diffY;

                    if (cData.log[indexOfGrid] == null)
                    {
                        //neighboors.push({ swtc: 1, node: null, sgp: pos, iot: indexer, iog: indexOfGrid});
                        extraTiles.push({ sgp: pos, iot: indexer, docg: gridData});
                    }
                    else
                    {
                        neighboors.push({ swtc: 0, node: cData.log[indexOfGrid].grid[indexer], sgp: null, iot: null, iog: null});
                    }
                }
            }
        }
        return { neighboors: neighboors, extraTiles: extraTiles };
    },

    SetAllSidesUnwalkableGridIndex: function (node, gridWorldSize, cData, initialPos)
    {
        var neighboors = [];
        var extraTiles = [];
        for (var x = -1; x <= 1; x++) {
            for (var y = -1; y <= 1; y++) {
                if (x == 0 && y == 0) {
                    continue;
                }

                var xpos = Math.round(node.worldPosition.x + x);
                var ypos = Math.round(node.worldPosition.y + y);

                var pos = { x: xpos, y: ypos };

                var gridData = SC_PathFindUtilities_cc_4.getNewGridIndex(initialPos, pos, gridWorldSize, cData.objt.sid);

                var indexOfGrid = gridData.index;

                var xpos = Math.round(node.worldPosition.x + x);
                var ypos = Math.round(node.worldPosition.y + y);

                var pos = { x: xpos, y: ypos };

                if (xpos >= node.gridPos.x - gridWorldSize.xL && xpos <= node.gridPos.x + gridWorldSize.xR &&
                    ypos >= node.gridPos.y - gridWorldSize.yB && ypos <= node.gridPos.y + gridWorldSize.yT) {
                    var gridTileX = node.gridTileX + x;
                    var gridTileY = node.gridTileY + y;

                    if (gridTileX < 0) {
                        gridTileX = (gridWorldSize.xR) + (gridTileX * -1);
                    }

                    if (gridTileY < 0) {
                        gridTileY = (gridWorldSize.yT) + (gridTileY * -1);
                    }
                    var index = ((gridTileX) * (gridWorldSize.xL + gridWorldSize.xR + 1)) + (gridTileY);

                    //SET STATION TILE NEIGHBOORS TO UNWALKABLES
                    //SET STATION TILE NEIGHBOORS TO UNWALKABLES
                    //cData.log[node.gridIndex].grid[index].closed = 1;
                    //SET STATION TILE NEIGHBOORS TO UNWALKABLES
                    //SET STATION TILE NEIGHBOORS TO UNWALKABLES

                    neighboors.push({ swtc: 0, node: cData.log[node.gridIndex].grid[index], sgp: null, iot: null, iog: null });
                }
                else {
                    var gridData = SC_PathFindUtilities_cc_4.getNewGridIndex(initialPos, pos, gridWorldSize, cData.objt.sid);

                    var indexOfGrid = gridData.index;

                    var diffX = Math.round(Math.abs(Math.abs(pos.x) - Math.abs(gridData.gridData.x)));
                    var diffY = Math.round(Math.abs(Math.abs(pos.y) - Math.abs(gridData.gridData.y)));

                    var starterGriderPos = { x: gridData.gridData.x, y: gridData.gridData.y };

                    //console.PrintError("iog: " + indexOfGrid);

                    //var idObj = generator.AddSpecialObject(cData.objt.sid, gridData.gridData.x, gridData.gridData.y, "waypoint_02", 0);
                    //
                    //if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                    //    var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                    //    crates.push(idObj);
                    //    storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    //}
                    //else {
                    //    var crates = [];
                    //    crates.push(idObj);
                    //    storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    //}

                    if (pos.x < starterGriderPos.x) {
                        diffX = (gridWorldSize.xR) + diffX;
                    }
                    else if (pos.x > starterGriderPos.x) {
                        //console.PrintError("testing");
                        diffX = diffX;
                    }
                    else {
                        diffX = 0;
                    }

                    if (pos.y < starterGriderPos.y) {
                        diffY = (gridWorldSize.yT) + diffY;
                    }
                    else if (pos.y > starterGriderPos.y) {
                        diffY = diffY;
                    }
                    else {
                        diffY = 0;
                    }

                    var indexer = diffX * (gridWorldSize.xL + gridWorldSize.xR + 1) + diffY;

                 

                    if (cData.log[indexOfGrid] == null) {




                        //neighboors.push({ swtc: 1, node: null, sgp: pos, iot: indexer, iog: indexOfGrid});
                        extraTiles.push({ sgp: pos, iot: indexer, docg: gridData }); //, closed:1 
                    }
                    else {

                        //SET STATION TILE NEIGHBOORS TO UNWALKABLES
                        //SET STATION TILE NEIGHBOORS TO UNWALKABLES
                        //cData.log[indexOfGrid].grid[indexer].closed = 1;
                        //SET STATION TILE NEIGHBOORS TO UNWALKABLES
                        //SET STATION TILE NEIGHBOORS TO UNWALKABLES


                        neighboors.push({ swtc: 0, node: cData.log[indexOfGrid].grid[indexer], sgp: null, iot: null, iog: null });
                    }
                }
            }
        }
        return { neighboors: neighboors, extraTiles: extraTiles };
    }
};