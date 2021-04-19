using(console);

var SC_AI_PathFind_Mining_GetGridWorldPos_5 =
{
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
            dividerDecimal = 0.16666666666666666666666666666667; //original 34 digits starting with 0 and including the dot 0.16666666666666666666666666666667  octal numbers are not allowed in strict mode and we get a console error and the server doesn't start in singleplayer. it must probably not work in multiplayer either.
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

        return { x: currentGridPosX, y: currentGridPosY, gridX: flooredRemainsDivX, gridY: flooredRemainsDivY }; // 
    }
};





























