using(console);

var SC_PathFindUtilities_Mining_1 =
{
    npcCheckDistance: function (nodeA, nodeB) {
        var dstX = Math.abs((nodeA.x) - (nodeB.x));
        var dstZ = Math.abs((nodeA.y) - (nodeB.y));

        if (dstX > dstZ)
            return 14 * dstZ + 10 * (dstX - dstZ);
        return 14 * dstX + 10 * (dstZ - dstX);
    },

    /*doesContain: function (a, obj)
    {
        if (a.length > 0)
        {
            for (var i = 0; i < a.length; i++)
            {
                //if (a[i].boolWalk == null || obj == null)
                //{
                //    continue;
                //}
                var aax = a[i].worldPosition.x;
                var aay = a[i].worldPosition.y;

                var bbx = obj.worldPosition.x;
                var bby = obj.worldPosition.y;

                if (aax === bbx && aay === bby) {
                    return 1;
                }
            }
            return 0;
        }
        else {
            return 0;
        }
    },*/

    /*doesContainPos: function (a, obj) {
        if (a.length > 0) {
            for (var i = 0; i < a.length; i++) {
                var aax = a[i].worldPosition.x;
                var aay = a[i].worldPosition.y;

                var bbx = obj.x;
                var bby = obj.y;

                if (aax === bbx && aay === bby) {
                    return 1;
                }
            }
            return 0;
        }
        else {
            return 0;
        }
    },

    doesContainGridIndex: function (a, obj) {
        for (var i = 0; i < a.length; i++) {
            var aax = a[i].gridIndex;
            var bbx = obj.gridIndex;

            if (aax === bbx) {
                return 1;
            }
        }
        return 0;
    },*/

    getGridWorldPosition: function (initialPos, seekerPos, gridWorldSize, sid)  //yep still fail. yep. yeah... godamnit.
    {
        var diffX = Math.round(Math.abs(Math.abs(seekerPos.x) - Math.abs(initialPos.x)));
        var diffY = Math.round(Math.abs(Math.abs(seekerPos.y) - Math.abs(initialPos.y)));

        var currentGridPosX;
        var currentGridPosY;
        var dividerDecimal = 0;

        if ((gridWorldSize.xL + gridWorldSize.xR + 1) == 5) {

            dividerDecimal = 0.2;
        }
        else if ((gridWorldSize.xL + gridWorldSize.xR + 1) == 10)
        {
            dividerDecimal = 0.1;
        }
        else if ((gridWorldSize.xL + gridWorldSize.xR + 1) == 20)
        {
            dividerDecimal = 0.05;
        }
        else if ((gridWorldSize.xL + gridWorldSize.xR + 1) == 8)
        {
            dividerDecimal = 0.125;
        }
        else if ((gridWorldSize.xL + gridWorldSize.xR + 1) == 6)
        {
            dividerDecimal = 0.16666666666666666666666666666667;
        }
        else if ((gridWorldSize.xL + gridWorldSize.xR + 1) == 4)
        {
            dividerDecimal = 0.25;
        }


        var totalRemainsDivX = (diffX * dividerDecimal);
        var flooredRemainsDivX = Math.floor(totalRemainsDivX);
        var lastRemainsDivX = totalRemainsDivX - flooredRemainsDivX;

        var testX00 = diffX - gridWorldSize.xL - (flooredRemainsDivX * (gridWorldSize.xL + gridWorldSize.xR + 1));
        var testX01 = diffX - gridWorldSize.xR - (flooredRemainsDivX * (gridWorldSize.xL + gridWorldSize.xR + 1));

        if (seekerPos.x < initialPos.x) {
            if (testX00 > 0) {
                flooredRemainsDivX = flooredRemainsDivX + 1;
            }
            currentGridPosX = Math.round(initialPos.x - ((gridWorldSize.xL + gridWorldSize.xR + 1) * flooredRemainsDivX));
        }
        else if (seekerPos.x > initialPos.x) {
            if (testX01 > 0) {
                flooredRemainsDivX = flooredRemainsDivX + 1;
            }
            currentGridPosX = Math.round(initialPos.x + ((gridWorldSize.xL + gridWorldSize.xR + 1) * flooredRemainsDivX));
        }
        else {
            currentGridPosX = Math.round(initialPos.x);
        }

        var totalRemainsDivY = (diffY * dividerDecimal);
        var flooredRemainsDivY = Math.floor(totalRemainsDivY);
        var lastRemainsDivY = totalRemainsDivY - flooredRemainsDivY;

        var testY00 = diffY - gridWorldSize.yB - (flooredRemainsDivY * (gridWorldSize.xL + gridWorldSize.xR + 1));
        var testY01 = diffY - gridWorldSize.yT - (flooredRemainsDivY * (gridWorldSize.xL + gridWorldSize.xR + 1));

        if (seekerPos.y < initialPos.y) {
            if (testY00 > 0) {
                flooredRemainsDivY = flooredRemainsDivY + 1;
            }
            currentGridPosY = Math.round(initialPos.y - ((gridWorldSize.yB + gridWorldSize.yT + 1) * flooredRemainsDivY));
        }
        else if (seekerPos.y > initialPos.y) {
            if (testY01 > 0) {
                flooredRemainsDivY = flooredRemainsDivY + 1;
            }
            currentGridPosY = Math.round(initialPos.y + ((gridWorldSize.yB + gridWorldSize.yT + 1) * flooredRemainsDivY));
        }
        else {
            currentGridPosY = Math.round(initialPos.y);
        }

        /*var idObj = generator.AddSpecialObject(sid, currentGridPosX, currentGridPosY, "waypoint_03", 0);

        if (storage.IsSetGlobal("crates_" + sid)) {
            var crates = storage.GetGlobal("crates_" + sid);
            crates.push(idObj);
            storage.SetGlobal("crates_" + sid, crates);
        }
        else {
            var crates = [];
            crates.push(idObj);
            storage.SetGlobal("crates_" + sid, crates);
        }*/
        return { x: currentGridPosX, y: currentGridPosY, gridX: flooredRemainsDivX, gridY: flooredRemainsDivY }; // 
    },



    getNewGridIndex: function (initialPos, seekerPos, gridWorldSize, sid) {
        var gridData = SC_PathFindUtilities_Mining_1.getGridWorldPosition(initialPos, seekerPos, gridWorldSize, sid);

        var testX = 0;
        var testY = 0;
        var minY = 0;
        var maxY = 0;

        var minX = 0;
        var maxX = 0;

        var index = 0;

        if (gridData.x < initialPos.x) {
            testX = -gridData.gridX;
        }
        else {
            testX = gridData.gridX;
        }

        if (gridData.y < initialPos.y) {
            testY = -gridData.gridY;
        }
        else {
            testY = gridData.gridY;
        }

        var someAdder = 0;

        if (gridData.gridX >= gridData.gridY)
        {
            if (testX <= 0)
            {
                minX = gridData.gridX; //2
                maxX = gridData.gridX; //2

                maxY = maxX; //2
                minY = maxY-1; //1
              

                var currentContainedGrids = (minX + maxX) * (minY + maxY + 1); //2+2*2+1+1=4*4=16

                if (testY >= 0) // REVISED
                {
                    //61
                    //someAdder = (minY + gridData.gridY + 1); //4+1+1 = 6
                    //index = currentContainedGrids - (minY + maxY+1) + someAdder;//63-8 +6 = 55+6 = 61

                    //index 3
                    someAdder = (minY + gridData.gridY + 1); //
                    var tot = minY + maxY + 1;
                    index = currentContainedGrids - (tot) + someAdder;

                    //index 14
                    someAdder = (minY + gridData.gridY + 1); //1+0+1 = 2
                    var tot = minY + maxY + 1; // 
                    index = currentContainedGrids - (tot) + someAdder; // 16 -

                }
                else // REVISED
                {
                    //index 2
                    //someAdder = (minY - gridData.gridY + 1); //0-1+1 = 0
                    //var tot = minY + maxY + 1;
                    //index = currentContainedGrids - (tot) + someAdder;// 4 - 2 + 0 = 2

                    //index 59
                    someAdder = (minY - gridData.gridY + 1); //3-1+1 = 3

                    var tot = minY + maxY + 1;
                    index = currentContainedGrids - (tot) + someAdder;
                }
            }
            else
            {
                minX = gridData.gridX; //3
                maxX = gridData.gridX; //3

                minY = minX; //3
                maxY = maxX; //3

                var currentContainedGrids = (minX + maxX+1) * (minY + maxY); // 42

                if (testY >= 0)
                {
                    // index 7 
                    someAdder = (minY + gridData.gridY + 1);

                    var tot = minY + maxY + 1;
                    index = currentContainedGrids + (tot) - someAdder;
                }
                else
                {
                    //index47
                    someAdder = (minY - gridData.gridY + 1); // 3-2+1 = 2
                    var tot = minY + maxY + 1;
                    index = currentContainedGrids + (tot) - someAdder; //42+7 = 49 - 2 = 47
                }
            }
        }
        else if (gridData.gridX < gridData.gridY)
        {
            if (testY <= 0) 
            {
                minY = gridData.gridY;//4
                maxY = gridData.gridY;//4

                minX = minY; //4
                maxX = minX-1; //3

                var currentContainedGrids = (minX + maxX) * (minY + maxY);//8*7 = 56

                var someAdder = 0;
                if (testX >= 0)
                {
                    //index 25  => totalGrids 
                    someAdder = (minX - gridData.gridX + 1); //3-2+1 = 2
                    var tot = minY + maxY + 1;
                    index = currentContainedGrids - (tot) + someAdder; //30-7 + 2 = 25

                    //index 1  => totalGrids 
                    //someAdder = (minX - gridData.gridX+1); //1-0+1  = 2
                    //index = currentContainedGrids - (minY + maxY + 1) + someAdder; //2-1-1-1 + 2
                }
                else
                {
                    //index 28 =>
                    someAdder = (minX + gridData.gridX + 1); //3+1+1 = 5
                    var tot = minY + maxY + 1; // 3+3+1 = 7
                    index = currentContainedGrids - (tot) + someAdder; //30-7 = 23 + 5 = 28

                    //index 53 =>
                    //someAdder = (minX + gridData.gridX + 1); // 4+1+1 = 6
                    //var tot = minY + maxY + 1; // 4*4*1 = 9
                    //index = currentContainedGrids - (tot) + someAdder;//56-9 = 47 + 6 = 53

                    //index 54 =>
                    //someAdder = (minX + gridData.gridX + 1); //
                    //var tot = minY + maxY + 1; //
                    //index = currentContainedGrids - (tot) + someAdder;//
                }
            }
            else //if (testY > 0) 
            {
                //test with index 41
                //minY = gridData.gridY + 1;

                minY = gridData.gridY; // 3
                maxY = gridData.gridY; // 3

                minX = minY; // 3
                maxX = maxY; // 3 

                var currentContainedGrids = (minX + maxX) * (minY + maxY); // 36 total grids.... meaning index 35 is the biggest

                var someAdder = 0;
                if (testX >= 0) {
                    //index 41
                    someAdder = (minX + gridData.gridX); // 3+2
                    index = currentContainedGrids + someAdder; // 36+5 = 41
                }
                else {
                    //index 38 
                    someAdder = (maxX - gridData.gridX); // 3-2
                    index = currentContainedGrids + someAdder;
                }
            }
        }

        return { gridData: gridData, index: index };

        //return index;
    },

    getCurrentGridSize: function (flooredRemainsDivX, flooredRemainsDivY) {
        var currentGridSize = 3;
        var halfOf = 1;

        if (flooredRemainsDivX > flooredRemainsDivY) {
            currentGridSize = flooredRemainsDivX + flooredRemainsDivX + 1;
            halfOf = flooredRemainsDivX;
        }
        else if (flooredRemainsDivY > flooredRemainsDivX) {
            currentGridSize = flooredRemainsDivY + flooredRemainsDivY + 1;
            halfOf = flooredRemainsDivY;
        }
        if (currentGridSize < 3) {
            currentGridSize = 3;
            halfOf = 1;
        }
        return { gridS: currentGridSize, gridSH: halfOf };
    },
};

/*function contains(a, obj) {
    for (var i = 0; i < a.length; i++) {
        if (a[i].worldPosition.x === obj.worldPosition.x && a[i].worldPosition.y === obj.worldPosition.y) {
            return 1;
        }
    }
    return 0;
}

function containsIndex(a, obj) {
    for (var i = 0; i < a.length; i++) {
        if ((a[i].worldPosition.x) == (obj.worldPosition.x) && (a[i].worldPosition.y) == (obj.worldPosition.y)) {
            return i;
        }
    }
    return -1;
}


function contains(a, b) {
    for (var i = 0; i < a.length; i++) {
        if (a[i].a.x === b.a.x && a[i].a.y === b.a.y) {
            return 1;
        }
    }
    return 0;
}*/




































