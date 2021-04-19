using(npc);
using(console);

var SC_Utilities =
{

    /*float AngleBetween(float x1, float y1, float x2, float y2)
{
    return atan2(y2 - y1, x2 - x1);
}

float DegreeToRadian(float angle)
{
    return PI * angle / 180.0f;
}

float RadianToDegree(float angle)
{
    return angle * (180.0f / PI);
}


*/
    //https: //stackoverflow.com/questions/13695317/rotate-a-point-around-another-point
    RotatePoint: function (pointToRotate, centerPoint, angleInDegrees) {
        var angleInRadians = angleInDegrees * (Math.PI / 180);
        var cosTheta = Math.cos(angleInRadians);
        var sinTheta = Math.sin(angleInRadians);

        var newX = (cosTheta * (pointToRotate.x - centerPoint.x) - sinTheta * (pointToRotate.y - centerPoint.y) + centerPoint.x);
        var newY = (sinTheta * (pointToRotate.x - centerPoint.x) + cosTheta * (pointToRotate.y - centerPoint.y) + centerPoint.y);

        var newPos = { x: newX, y: newY };

        return newPos;
    },


    DegreeToRadian: function (angle) {
        return Math.PI * angle / 180.0;
    },

    RadianToDegree: function (angle) {
        return angle * (180.0 / Math.PI);
    },


    ////https://stackoverflow.com/questions/1628386/normalise-orientation-between-0-and-360  //tvanfosson and
    normalizedegrees: function (radians) {

        var degrees = SC_Utilities.RadianToDegree(radians);
        degrees = degrees % 360;
        if (degrees < 0) {
            degrees += 360;
        }
        return SC_Utilities.DegreeToRadian(degrees);
    },

    //Sebastian Lague Youtube Pathfind Tutorial START
    npcCheckDistance: function (nodeA, nodeB) {
   
        if (nodeA == null || nodeB == null || nodeA.x == null || nodeA.y == null || nodeB.x == null || nodeB.y == null)
        {
            if (nodeA == null || nodeA.x == null || nodeA.y == null) {

                console.PrintError("coordinates nodeA are null");
            }
            if (nodeB == null || nodeB.x == null || nodeB.y == null) {

                console.PrintError("coordinates nodeB are null");
            }

            //console.PrintError("coordinates are null");
            return null;
        }
        var dstX = Math.abs((nodeA.x) - (nodeB.x));
        var dstZ = Math.abs((nodeA.y) - (nodeB.y));


        if (dstX > dstZ)
            return 14 * dstZ + 10 * (dstX - dstZ);
        return 14 * dstX + 10 * (dstZ - dstX);
    },
    //Sebastian Lague Youtube Pathfind Tutorial END

    Dot: function (aX, aY, bX, bY) {
        return (aX * bX) + (aY * bY);
    },

    GetDistancePow: function (x1, y1, x2, y2) {
        return Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
    },

    //static System.Random randomer = new System.Random();
    getSomeRandNum: function (min, max)//(float min, float max) 
    {
        var num = (Math.floor(Math.random() * max)) + 1; //999999999 // this will get a number between 1 and 99;
        num *= Math.floor(Math.random() * 2) == 1 ? 1 : -1; // this will add minus sign in 50% of cases
        if (num == 0) {
            return getSomeRandNum(min, max);
        }
        return num * min; // 0.000000001
    },


    //found on the gamedevstackexchangeforums or the unity3d forums
    getSomeRandNumThousandDecimal: function (min, max, negativeswtchzerofornot)//(float min, float max, float negativeswtchzerofornot)
    {
        var num = Math.floor(Math.random() * max) + 1; //999
        num *= Math.floor(Math.random() * 2) == 1 ? 1 : -1; // this will add minus sign in 50% of cases
        if (negativeswtchzerofornot == 1) {
            if (num == 0) {
                return getSomeRandNum(min, max);
            }
        }
        /*else
        {

        }*/
        return (num * min); //0.001f
    },

    ClampValue: function (value, min, max) {

        value = value % max;
        if (value < min) {
            return min;
        }
        else if (value > max) {
            return max;
        }

        return value;
    },

    GetDistance: function (a,b) {
        return Math.sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    },


    crossProd: function (a, b) {
        var cross = a.x * b.y - b.x * a.y;
        return cross;
    },

    NSEW: function (a, b, c) {
        return ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) > 0;
    },

    NSDIST: function (a, b, c) {
        return ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
    },

    EWDIST: function (a, b, c) {
        return ((b.y - a.y) * (c.x - a.x) - (b.x - a.x) * (c.y - a.y));
    },
    

    finder: function(cmp, arr, getter) {
        var val = getter(arr[0]);
        for (var i = 1; i < arr.length; i++) {
            val = cmp(val, getter(arr[i]));
        }
        return val;
    },

    //Got that from the internet from gamedev.stackOverflow if I remember correctly.
    contains: function (a, obj)
    {
        for (var i = 0; i < a.length; i++)
        {
            //console.PrintError(a[i].indexOf(obj));
            if (JSON.stringify(a[i]) === JSON.stringify(obj)) //a[i].valueOf() === obj.valueOf() //a[i].indexOf(obj) >= 0
            {
                return true;
            }
        }

        return false;
    },


    //Got that from the internet from gamedev.stackOverflow if I remember correctly.
    containsNReturnIndex: function (a, obj) {
        for (var i = 0; i < a.length; i++) {

            var aa = a[i] + "";
            var bb = obj + "";

            //console.PrintError(a[i].indexOf(obj));
            if (JSON.stringify(aa) === JSON.stringify(bb)) //a[i].valueOf() === obj.valueOf() //a[i].indexOf(obj) >= 0
            {
                var dataOut = { result: true, index: i };
                return dataOut;
            }
        }

        var dataOut = { result: false, index: -1 };
        return dataOut;
    },


    isAlphaOnly: function (a)
    {
        var b = '';
        for (var i = 0; i < a.length; i++)
        {
            if (a[i] >= 'A' && a[i] <= 'z')
            {

            }
            else
            {
                return 0;
            }
        }
        return 1;
    },





    //Got that from the internet from gamedev.stackOverflow if I remember correctly.
    alphaOnly: function(a) {
        var b = '';
        for (var i = 0; i < a.length; i++)
        {
            if (a[i] >= 'A' && a[i] <= 'z')
            {
                b += a[i];
            }         
        }
        return b;
    },

    //Got that from the internet from gamedev.stackOverflow if I remember correctly.
    numberOnly: function(a) {
        var b = '';
        for (var i = 0; i < a.length; i++) {
            if (a[i] >= '0' && a[i] <= '9') b += a[i];
        }
        return b;
    },

    setBehavior: function (id, isEvading)
    {
        if (isEvading == 1) {
            npc.SetBehavior(id, "avoid_asteroids", true);
            npc.SetBehavior(id, "avoid_ships", true);
            npc.SetBehavior(id, "avoid_bases", true);
            npc.SetBehavior(id, "avoid_debris", true);
        }
        else {
            npc.SetBehavior(id, "avoid_asteroids", false);
            npc.SetBehavior(id, "avoid_ships", false);
            npc.SetBehavior(id, "avoid_bases", false);
            npc.SetBehavior(id, "avoid_debris", false);
        }
    }
};















































/*//https://stackoverflow.com/questions/13695317/rotate-a-point-around-another-point
function RotatePoint(pointToRotate, centerPoint, angleInDegrees)
{
    var angleInRadians = angleInDegrees * (Math.PI / 180);
    var cosTheta = Math.cos(angleInRadians);
    var sinTheta = Math.sin(angleInRadians);

    var newX = (cosTheta * (pointToRotate.x - centerPoint.x) - sinTheta * (pointToRotate.y - centerPoint.y) + centerPoint.x);
    var newY = (sinTheta * (pointToRotate.x - centerPoint.x) + cosTheta * (pointToRotate.y - centerPoint.y) + centerPoint.y);

    var newPos = { x: newX, y: newY };

    return newPos;
}




function Dot(aX, aY, bX, bY) {
    return (aX * bX) + (aY * bY);
}

function finder(cmp, arr, getter) {
    var val = getter(arr[0]);
    for (var i = 1; i < arr.length; i++) {
        val = cmp(val, getter(arr[i]));
    }
    return val;
}

function GetDistance(x1, y1, x2, y2) {
    return Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
}

function contains(a, obj) {
    for (var i = 0; i < a.length; i++) {
        if (a[i] === obj) {
            return true;
        }
    }
    return false;
}

function alphaOnly(a) {
    var b = '';
    for (var i = 0; i < a.length; i++) {
        if (a[i] >= 'A' && a[i] <= 'z') b += a[i];
    }
    return b;
}

function numberOnly(a) {
    var b = '';
    for (var i = 0; i < a.length; i++) {
        if (a[i] >= '0' && a[i] <= '9') b += a[i];
    }
    return b;
}





function RotatePoint(pointToRotate, centerPoint, angleInDegrees) {
    var angleInRadians = angleInDegrees * (Math.PI / 180);
    var cosTheta = Math.cos(angleInRadians);
    var sinTheta = Math.sin(angleInRadians);

    var newX = (cosTheta * (pointToRotate.x - centerPoint.x) - sinTheta * (pointToRotate.y - centerPoint.y) + centerPoint.x);
    var newY = (sinTheta * (pointToRotate.x - centerPoint.x) + cosTheta * (pointToRotate.y - centerPoint.y) + centerPoint.y);

    var newPos = { x: newX, y: newY };

    return newPos;
}
function setBehavior(id, isEvading) {
    if (isEvading == 1) {
        npc.SetBehavior(id, "avoid_asteroids", true);
        npc.SetBehavior(id, "avoid_ships", true);
        npc.SetBehavior(id, "avoid_bases", true);
        npc.SetBehavior(id, "avoid_debris", true);
    }
    else {
        npc.SetBehavior(id, "avoid_asteroids", false);
        npc.SetBehavior(id, "avoid_ships", false);
        npc.SetBehavior(id, "avoid_bases", false);
        npc.SetBehavior(id, "avoid_debris", false);
    }
}


//https://stackoverflow.com/questions/2333292/cross-product-of-2-2d-vectors
function crossProd(a, b) {
    var cross = a.x * b.y - b.x * a.y;
    return cross;
}


function NSEW(a, b, c) {
    return ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) > 0;
}

function NSDIST(a, b, c) {
    return ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
}

function EWDIST(a, b, c) {
    return ((b.y - a.y) * (c.x - a.x) - (b.x - a.x) * (c.y - a.y));
}*/




