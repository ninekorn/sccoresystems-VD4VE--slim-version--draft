cbuffer MatrixBuffer : register(b0)
{
	matrix worldMatrix;
	matrix viewMatrix;
	matrix projectionMatrix;
};

struct VertexInputType
{
    float4 position : POSITION;
    float2 tex : TEXCOORD0;
	float4 color : COLOR;
	float3 normal : NORMAL;
	float4 instancePosition : POSITION1;
	float4 instanceRadRot : POSITION2;
	float4 instanceRadRotRIGHT : POSITION3;
	float4 instanceRadRotUP : POSITION4;
};

struct PixelInputType
{
    float4 position : SV_POSITION;
    float2 tex : TEXCOORD0;
	float4 color : COLOR;
	float3 normal : NORMAL;
	float4 instancePosition : POSITION1;
	float4 instanceRadRot : POSITION2;
	float4 instanceRadRotRIGHT : POSITION3;
	float4 instanceRadRotUP : POSITION4;
};

static const float PI = 3.1415926535897932384626433832795f;
static const float3 world_forward = float3(0,0,1);
static const float3 world_backward = float3(0,0,-1);
static const float3 world_right = float3(1,0,0);
static const float3 world_left = float3(-1,0,0);
static const float3 world_up = float3(0,1,0);
static const float3 world_down = float3(0,-1,0);

float AngleBetween(float x1, float y1, float x2, float y2)
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

////https://stackoverflow.com/questions/1628386/normalise-orientation-between-0-and-360  //tvanfosson and
float _normalize_degrees(float radians)
{
	float degrees = RadianToDegree(radians);
	degrees = degrees % 360;
	if (degrees < 0)
	{
		degrees += 360;
	}
	return DegreeToRadian(degrees);
}


/*float2 rotateClockWise(float2 pointToRotate,float2 centerPoint,float angleInRadians)
{	    
	float cosTheta = cos(angleInRadians);
    float sinTheta = sin(angleInRadians);	
    float newX = (cosTheta * (pointToRotate.x - centerPoint.x) - sinTheta * (pointToRotate.y - centerPoint.y) + centerPoint.x);
    float newY = (sinTheta * (pointToRotate.x - centerPoint.x) + cosTheta * (pointToRotate.y - centerPoint.y) + centerPoint.y);
    return float2(newX,newY);
}

float2 rotateCounterClockWise(float2 pointToRotate,float2 centerPoint,float radians)
{	    
	//float cosTheta = cos(angleInRadians);
    //float sinTheta = sin(angleInRadians);	
    float newX = centerPoint.x+(cos(radians) * (pointToRotate.x - centerPoint.x) -  sin(radians)  * (pointToRotate.y - centerPoint.y));
	float newY = centerPoint.y+ ((-sin(radians))  * (pointToRotate.x - centerPoint.x) + cos(radians) * (pointToRotate.y - centerPoint.y));
	return float2(newX,newY);
}*/


//https://stackoverflow.com/questions/22818531/how-to-rotate-2d-vector => Martin Meeser  //double[] result = new double[2];
//=> My Version
float2 SC_RotateVector2d(float2 pointToRotate, float2 centerPoint, float radians)
{
    float cosTheta = cos(radians); // x
    float sinTheta = sin(radians);	//y
    float newX = (cosTheta * (pointToRotate.x - centerPoint.x) - sinTheta * (pointToRotate.y - centerPoint.y) + centerPoint.x);
    float newY = (sinTheta * (pointToRotate.x - centerPoint.x) + cosTheta * (pointToRotate.y - centerPoint.y) + centerPoint.y);
    return float2(newX,newY);

	//float cosTheta = cos(radians);
	//float sinTheta = sin(radians);
	//float newX = (cosTheta * (pointToRotate.x) - sinTheta * (pointToRotate.y));
	//float newY = (sinTheta * (pointToRotate.y) + cosTheta * (pointToRotate.x));
	//return float2(newX, newY);
	//float newX = centerPoint.x+(cos(radians) * (pointToRotate.x - centerPoint.x) -  sin(radians)  * (pointToRotate.y - centerPoint.y));
	//float newY = centerPoint.y+ ((sin(radians))  * (pointToRotate.x - centerPoint.x) + cos(radians) * (pointToRotate.y - centerPoint.y));
	//return float2(newX, newY);
}

/*//float radians = DegreeToRadian(degrees);
//float cosTheta = cos(radians);
//float sinTheta = sin(radians);
//float newPointRotX = input_vertex.x * cos(radians);
//float newPointRotY = input_vertex.y * sin(radians);

//float newPointRotX = (((input_vertex.x- centerPoint.x) * (cos(radians))) - ((input_vertex.y- centerPoint.y) * (sin(radians))))+ centerPoint.x;
//float newPointRotY = (((input_vertex.x- centerPoint.x) * (sin(radians))) + ((input_vertex.y- centerPoint.y) * (cos(radians))))+ centerPoint.y;

//float newX = ((cosTheta * (pointToRotate.x - centerPoint.x) - sinTheta * (pointToRotate.y - centerPoint.y)) + centerPoint.x);
//float newY = ((sinTheta * (pointToRotate.x - centerPoint.x) + cosTheta * (pointToRotate.y - centerPoint.y)) + centerPoint.y);
//float newX = cos(radians);
//float newY = sin(radians);
//float newX = centerPoint.x+(cos(radians)*(pointToRotate.x -centerPoint.x) + sin(radians) * (pointToRotate.y -centerPoint.y));
//float newY = centerPoint.y+(-sin(radians)*(pointToRotate.x - centerPoint.x) + cos(radians) * (pointToRotate.y -centerPoint.y));
//float newX = centerPoint.x+(cos(radians) * (pointToRotate.x - centerPoint.x) -  sin(radians)  * (pointToRotate.y - centerPoint.y));
//float newY = centerPoint.y+ ((-sin(radians))  * (pointToRotate.x - centerPoint.x) + cos(radians) * (pointToRotate.y - centerPoint.y));
*/


float distance(float2 a,float2 b) 
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

//https://stackoverflow.com/questions/13458992/angle-between-two-vectors-2d
//https://referencesource.microsoft.com/#WindowsBase/Base/System/Windows/Vector.cs,102
float AngleBetweener(float v1x,float v1y, float v2x,float v2y)
{
    float sin = v1x * v2y - v2x * v1y;  
    float cos = v1x * v2x + v1y * v2y;

    return atan2(sin, cos) * (180.0f / PI);
}

float distanceTo(float3 vector1, float3 vector2) 
{
    return sqrt(((vector1.x - vector2.x)*(vector1.x - vector2.x)) + ((vector1.y - vector2.y)*(vector1.y - vector2.y)) + ((vector1.z - vector2.z)*(vector1.z - vector2.z)));
}

float3 SphericalToCartesian(float radius, float polar, float elevation) //xyz
{
    float a = radius * cos(elevation);
    float x = a * cos(polar);
    float y = radius * sin(elevation);
    float z = a * sin(polar);

	return float3(x,y,z);
}

static float4 mod_input_vertex_pos;

static float3 forwardDir;
static float3 rightDir;
static float3 upDir;

static float3 MOVINGPOINT;
static float3 vertPos;
static float diffX;
static float diffY;
static float diffZ;

PixelInputType TextureVertexShader(VertexInputType input)
{
	PixelInputType output;
    
	input.position.w = 1.0f;

	mod_input_vertex_pos = input.position;

	mod_input_vertex_pos.x += input.instancePosition.x;
	mod_input_vertex_pos.y += input.instancePosition.y;
	mod_input_vertex_pos.z += input.instancePosition.z;
	mod_input_vertex_pos.w = 1.0f;

	forwardDir = float3(input.instanceRadRot.x, input.instanceRadRot.y, input.instanceRadRot.z);
	rightDir = float3(input.instanceRadRotRIGHT.x, input.instanceRadRotRIGHT.y, input.instanceRadRotRIGHT.z); //cross(world_up,float3(input.instanceRadRot.x, input.instanceRadRot.y, input.instanceRadRot.z));//float3(input.instanceRadRot.x, input.instanceRadRot.y, input.instanceRadRot.z); //world_forward;//
	upDir = float3(input.instanceRadRotUP.x, input.instanceRadRotUP.y, input.instanceRadRotUP.z);

	MOVINGPOINT = float3(input.instancePosition.x, input.instancePosition.y, input.instancePosition.z);
	vertPos = float3(mod_input_vertex_pos.x, mod_input_vertex_pos.y, mod_input_vertex_pos.z);

	diffX = vertPos.x - input.instancePosition.x;
	diffY = vertPos.y - input.instancePosition.y;
	diffZ = vertPos.z - input.instancePosition.z;

	MOVINGPOINT = MOVINGPOINT + -(rightDir * diffX);
	MOVINGPOINT = MOVINGPOINT + -(upDir * diffY);
	MOVINGPOINT = MOVINGPOINT + -(forwardDir * diffZ);		

	input.position.x = MOVINGPOINT.x;
	input.position.y = MOVINGPOINT.y;
	input.position.z = MOVINGPOINT.z;

	output.position = mul(input.position, worldMatrix);
    output.position = mul(output.position, viewMatrix);
    output.position = mul(output.position, projectionMatrix);
   
	output.instancePosition.x = input.instancePosition.x;
	output.instancePosition.y = input.instancePosition.y;
	output.instancePosition.z = input.instancePosition.z;
	   
	output.instanceRadRot.x = input.instanceRadRot.x;
	output.instanceRadRot.y = input.instanceRadRot.y;
	output.instanceRadRot.z = input.instanceRadRot.z;

	output.tex = input.tex;
    output.color = input.color;	

	output.normal = input.normal;
	float xer = dot(output.normal, float3(1, 0.25, 0.4));		//float3(0.75, 0.50, 0.25));  //float3(1, 0.25, 0.4));   //float3(0.35, 0.15, 0.25));
	
	//xer *= 10.25;

	float x = dot(input.normal,input.instanceRadRotUP);// input.instanceRadRotUP);	

	x = x * 0.5 - 0.5;

	/*if(x < 0.95)
	{
		x = 0.95;
	}*/

	float x0 = input.color.x;
	float y0 = input.color.y;
	float z0 = input.color.z;
	float w0 = input.color.w;

	//float xx = xer;
	//float3 color = lerp(float3(input.color.x*0.90*xx, input.color.y*0.90*xx, input.color.z*0.90*xx), float3(input.color.x*0.95*xx, input.color.y*0.95*xx, input.color.z*0.95*xx), x);
	//input.color.xyz = color;
	//input.color.w = 1.0;
	//output.color = input.color;

    return output;
}




	//input.color = input.color;
	//xer = xer * 0.5 - 0.5;
	//world_up

















	//output.normal = mul(input.normal, (float3x3)worldMatrix);
    //output.normal = normalize(output.normal);

	//float HYP = distance(float2(vertPos.x,vertPos.y),float2(input.instancePosition.x, input.instancePosition.y));	
	//float ADJ = cos(angle) * HYP;
	//float OPP = sin(angle) * HYP;

	//MOVINGPOINT = MOVINGPOINT + (rightDir * ADJ);
	//MOVINGPOINT = MOVINGPOINT + (upDir * OPP);
	//MOVINGPOINT = MOVINGPOINT + (forwardDir * HYP);		
	

	//float4 dir_ori_to_mod_vertex = mod_input_vertex_pos - input.instancePosition;
	//float angle = AngleBetweener(dir_ori_to_mod_vertex.x,dir_ori_to_mod_vertex.y,1,0);	
	//float OPP = sin(angle) * HYP; // y length
	//float ADJ = cos(angle) * HYP;
	//float TAN = atan(OPP/ADJ);

	//float ADJ = cos(RadianToDegree(angle)) * HYP; // x length
	//float OPP = sin(RadianToDegree(angle)) * firstTriangleHYP; // y length
	//float4 dir_ori_to_mod_vertex = mod_input_vertex_pos - input.instancePosition;
	//float AngleWhenObjectHasntMoved0 = AngleBetweener(dir_ori_to_mod_vertex.x,dir_ori_to_mod_vertex.y,1,0);
	//float _x = (firstTriangleHYP * cos(AngleWhenObjectHasntMoved0)); // Adj		
	//MOVINGPOINT = MOVINGPOINT + (rightDir * ADJ);
	//MOVINGPOINT = MOVINGPOINT + (upDir * OPP);
	//MOVINGPOINT = MOVINGPOINT + (forwardDir * HYP);			
	//input.position.x = MOVINGPOINT.x;
	//input.position.y = MOVINGPOINT.y;
	//input.position.z = MOVINGPOINT.z;



	/*float firstTriangleHyp = distance(float2(vertPos.x,vertPos.y),float2(input.instancePosition.x, input.instancePosition.y));

	float4 dir_ori_to_mod_vertex = mod_input_vertex_pos - input.instancePosition;

	float AngleWhenObjectHasntMoved0 = AngleBetweener(dir_ori_to_mod_vertex.x,dir_ori_to_mod_vertex.y,1,0);
	float _x = (firstTriangleHyp * cos(AngleWhenObjectHasntMoved0)); // Adj

	MOVINGPOINT = MOVINGPOINT + -(rightDir * _x);

	//float secondTrig = distance(float2(vertPos.x,vertPos.y),float2(input.instancePosition.x, input.instancePosition.y));
	float AngleWhenObjectHasntMoved1 = AngleBetweener(dir_ori_to_mod_vertex.x,dir_ori_to_mod_vertex.y,0,1);
	float _y = (firstTriangleHyp * cos(AngleWhenObjectHasntMoved1)); // Adj
	
	MOVINGPOINT = MOVINGPOINT + -(upDir * _y);

	float thirdTrig = distance(float2(vertPos.z,vertPos.y),float2(input.instancePosition.z, input.instancePosition.y));
	//firstTriangleHyp = distance(float2(MOVINGPOINT.z,MOVINGPOINT.y),float2(input.instancePosition.z, input.instancePosition.y));
	float AngleWhenObjectHasntMoved2 = AngleBetweener(dir_ori_to_mod_vertex.z,dir_ori_to_mod_vertex.y,1,0);
	float _z = (thirdTrig * cos(AngleWhenObjectHasntMoved2)); // Adj
	
	MOVINGPOINT = MOVINGPOINT + -(forwardDir * _z);
		
	//input.position.x = _x + input.instancePosition.x;
	//input.position.y = _y + input.instancePosition.y;
	//input.position.z = _z + input.instancePosition.z;
		
	input.position.x = MOVINGPOINT.x;// + input.instancePosition.x;
	input.position.y = MOVINGPOINT.y;// + input.instancePosition.y;
	input.position.z = MOVINGPOINT.z;// + input.instancePosition.z;

	//float adjacent_tri_0 = cos(AngleWhenObjectHasntMoved0) * firstTriangleHyp; // x length
	//float opposite_tri_0 = sin(AngleWhenObjectHasntMoved0) * firstTriangleHyp; // y length
	*/

























	//VERTEXPOS = VERTEXPOS + (upDir * opposite_tri_0);

	//float thirdTrig = distance(float2(vertPos.z,vertPos.y),float2(input.instancePosition.z, input.instancePosition.y));
	//firstTriangleHyp = distance(float2(VERTEXPOS.z,VERTEXPOS.y),float2(input.instancePosition.z, input.instancePosition.y));
	//float AngleWhenObjectHasntMoved2 = AngleBetweener(dir_ori_to_mod_vertex.z,dir_ori_to_mod_vertex.y,1,0);
	//float _z = (thirdTrig * cos(AngleWhenObjectHasntMoved2)); // Adj
	
	//VERTEXPOS = VERTEXPOS + -(forwardDir * _z);
		
	//input.position.x = _x + input.instancePosition.x;
	//input.position.y = _y + input.instancePosition.y;
	//input.position.z = _z + input.instancePosition.z;
		
	//input.position.x = VERTEXPOS.x;// + input.instancePosition.x;
	//input.position.y = VERTEXPOS.y;// + input.instancePosition.y;
	//input.position.z = VERTEXPOS.z;// + input.instancePosition.z;


	/*float3 currentDirToVertex = VERTEXPOS - float3(input.instancePosition.x,input.instancePosition.y,input.instancePosition.z);
	float secondTrig = distance(float2(vertPos.x,vertPos.y),float2(input.instancePosition.x, input.instancePosition.y));
	float AngleWhenObjectHasntMoved1 = AngleBetweener(dir_ori_to_mod_vertex.x,dir_ori_to_mod_vertex.y,0,1);
	float _y = (secondTrig * cos(AngleWhenObjectHasntMoved1)); // Adj

	VERTEXPOS = VERTEXPOS + -(upDir * _y);

	float thirdTrig = distance(float2(vertPos.z,vertPos.y),float2(input.instancePosition.z, input.instancePosition.y));
	//firstTriangleHyp = distance(float2(VERTEXPOS.z,VERTEXPOS.y),float2(input.instancePosition.z, input.instancePosition.y));
	float AngleWhenObjectHasntMoved2 = AngleBetweener(dir_ori_to_mod_vertex.z,dir_ori_to_mod_vertex.y,1,0);
	float _z = (thirdTrig * cos(AngleWhenObjectHasntMoved2)); // Adj
	
	VERTEXPOS = VERTEXPOS + -(forwardDir * _z);
		
	//input.position.x = _x + input.instancePosition.x;
	//input.position.y = _y + input.instancePosition.y;
	//input.position.z = _z + input.instancePosition.z;
		
	input.position.x = VERTEXPOS.x;// + input.instancePosition.x;
	input.position.y = VERTEXPOS.y;// + input.instancePosition.y;
	input.position.z = VERTEXPOS.z;// + input.instancePosition.z;
	*/




	//////////////
	//////////////
	//////////////
	/*float4 dir_ori_to_mod_vertex = mod_input_vertex_pos - input.instancePosition;

	float AngleWhenObjectHasntMoved0 = AngleBetweener(dir_ori_to_mod_vertex.x,dir_ori_to_mod_vertex.y,1,0);
	float _x = (firstTriangleHyp * cos(AngleWhenObjectHasntMoved0)) + input.instancePosition.x; // Adj

	float AngleWhenObjectHasntMoved1 = AngleBetweener(dir_ori_to_mod_vertex.x,dir_ori_to_mod_vertex.y,0,1);
	float _y = (firstTriangleHyp * cos(AngleWhenObjectHasntMoved1)) + input.instancePosition.y; // Adj

	float AngleWhenObjectHasntMoved2 = AngleBetweener(dir_ori_to_mod_vertex.z,dir_ori_to_mod_vertex.y,1,0);
	float _z = (firstTriangleHyp * cos(AngleWhenObjectHasntMoved2)) + input.instancePosition.z; // Adj

	input.position.x = _x;
	input.position.y = _y;
	input.position.z = _z;*/
	//////////////
	//////////////
	//////////////

















	
	//float2 rightObject = float2(input.instanceRadRot.y,-input.instanceRadRot.x);
	//float2 upObject = float2(input.instanceRadRot.y, input.instanceRadRot.x);
	//float2 someDir = float2(input.instancePosition.x,input.instancePosition.y);

	//_x = input.instancePosition.x + (rightDir.x * _x);
	//_y = input.instancePosition.y + (forwardObject.y * _y);
	//_z = input.instancePosition.z + (forwardObject.y * _z);



	//float hypothenuse = distanceTo(vertPos, input.instancePosition); // correct length to input vertex.
	//float4 dir_ori_to_mod_vertex = mod_input_vertex_pos - input.instancePosition; // original direction to input vertex.
	
	//float4 dir_ori_object_direction_out_of_shader = input.instanceRadRot; // current object forward direction.
	//float2 sometester0 = float2(1,0);
	//float2 someTester1 = float2(dir_ori_object_direction_out_of_shader.x,dir_ori_object_direction_out_of_shader.y);
	//float someAngleYaw = AngleBetweener(someTester1.x,someTester1.y,sometester0.x,sometester0.y);
	
	//float AngleWhenObjectHasntMoved0 = AngleBetweener(dir_ori_to_mod_vertex.x,dir_ori_to_mod_vertex.y,1,0);
	//float AngleWhenObjectHasntMoved1 = AngleBetweener(dir_ori_object_direction_out_of_shader.x,dir_ori_object_direction_out_of_shader.y,1,0);

	//float _x = (hypothenuse * cos(AngleWhenObjectHasntMoved0))+ input.instancePosition.x; // Adj
	//float _x = (hypothenuse * cos(AngleWhenObjectHasntMoved1)) + input.instancePosition.x; // Adj

	//float hypothenuseForZ = distance(float2(vertPos.z,vertPos.x),float2( input.instancePosition.z, input.instancePosition.x))+ input.instancePosition.x;
	//float _y = (hypothenuse * sin(AngleWhenObjectHasntMoved0))+ input.instancePosition.y; // Opp
	//float _z = (hypothenuseForZ * tan(AngleWhenObjectHasntMoved0))+ input.instancePosition.z; // Opp

	//float3 normalizedVertexPos = float3(_x,mod_input_vertex_pos.y,mod_input_vertex_pos.z);
	//float3 normalizedVertexPos = float3(_x,_y,mod_input_vertex_pos.z);
	//float3 normalizedVertexPos = float3(mod_input_vertex_pos.x,mod_input_vertex_pos.y,mod_input_vertex_pos.z);

	//input.position.x = normalizedVertexPos.x;
	//input.position.y = normalizedVertexPos.y;
	//input.position.z = normalizedVertexPos.z;

	//float3 dirToNewVertPos = normalizedVertexPos - mod_input_vertex_centerOfRotation_pos;
	//float adjacent_tri_0 = cos(AngleWhenObjectHasntMoved) * hypothenuse; // x length
	//float opposite_tri_0 = sin(AngleWhenObjectHasntMoved) * hypothenuse; // y length

	//normalizedVertexPos.x = mod_input_vertex_centerOfRotation_pos.x + (dirToNewVertPos.x * (adjacent_tri_0));
	//normalizedVertexPos.y = mod_input_vertex_centerOfRotation_pos.y + (dirToNewVertPos.y * someTester);
	//normalizedVertexPos.z = mod_input_vertex_centerOfRotation_pos.z + (dirToNewVertPos.z * someTester);

	//float3 rightDir = float3(dir_ori_object_direction_out_of_shader.y,-dir_ori_object_direction_out_of_shader.x,dir_ori_object_direction_out_of_shader.z);
	//float3 leftDir = rightDir * -1;

	//float AngleWhenObjectHasntMoved = AngleBetweener(dir_ori_to_mod_vertex.x,dir_ori_to_mod_vertex.y,1,0);
	//float adjacent_tri_0 = cos(AngleWhenObjectHasntMoved) * hypothenuse; // x length
	//float opposite_tri_0 = sin(AngleWhenObjectHasntMoved) * hypothenuse; // y length

	//input.position.x = input.instancePosition.x + (input.instanceRadRot.x * hypothenuse);
	//input.position.y = input.instancePosition.x + (input.instanceRadRot.x * hypothenuse);
	//input.position.z = input.instancePosition.x + (input.instanceRadRot.z * hypothenuse);












	/*float2 dir_ori_object_direction_in_shader = float2(1,0);

	float AngleWhenObjectHasntMoved = AngleBetweener(dir_ori_to_mod_vertex.x,dir_ori_to_mod_vertex.y,1,0);

	float AngleWhenTheDataComesInPitch = AngleBetweener(dir_ori_object_direction_out_of_shader.x,dir_ori_object_direction_out_of_shader.y,1,0);
	float AngleWhenTheDataComesInYaw = AngleBetweener(dir_ori_object_direction_out_of_shader.z,dir_ori_object_direction_out_of_shader.x,0,1);
	float AngleWhenTheDataComesInRoll = AngleBetweener(dir_ori_object_direction_out_of_shader.z,dir_ori_object_direction_out_of_shader.y,0,1);

	float3 _xyz = SphericalToCartesian(AngleWhenTheDataComesInPitch,AngleWhenTheDataComesInYaw,AngleWhenTheDataComesInRoll);
	input.position.x = _xyz.x;
	input.position.y = _xyz.y;
	input.position.z = _xyz.z;*/






	//float pointFrontX = (1 * cos(AngleWhenObjectHasntMoved)) + mod_input_vertex_centerOfRotation_pos.x;
	//float pointFrontY = (1 * sin(AngleWhenObjectHasntMoved)) + mod_input_vertex_centerOfRotation_pos.y;

	//AngleWhenObjectHasntMoved = AngleBetweener(dir_ori_to_mod_vertex.z,dir_ori_to_mod_vertex.x,0,1);
	//float pointFrontZ = (1 * cos(AngleWhenObjectHasntMoved)) + input.instancePosition.z; //mod_input_vertex_pos.z;//

	//float adjacent_tri_0 = cos(AngleWhenObjectHasntMoved) * hypothenuse; // x length
	//float opposite_tri_0 = sin(AngleWhenObjectHasntMoved) * hypothenuse; // y length


	//float _x = (1 * cos(AngleWhenTheDataComesInPitch)) + mod_input_vertex_centerOfRotation_pos.x;
	//float _y = (1 * sin(AngleWhenTheDataComesInPitch)) + mod_input_vertex_centerOfRotation_pos.y;

	//float3 normalizedVertexPos = float3(_x,mod_input_vertex_pos.y,mod_input_vertex_pos.z);

	//float3 dirToNewVertPos = normalizedVertexPos - mod_input_vertex_centerOfRotation_pos;
	//float someTester = distanceTo(normalizedVertexPos,mod_input_vertex_centerOfRotation_pos);
	//dirToNewVertPos.x /= someTester;
	//dirToNewVertPos.y /= someTester;
	//dirToNewVertPos.z /= someTester;

	//normalizedVertexPos.x = mod_input_vertex_centerOfRotation_pos.x + (dirToNewVertPos.x * (adjacent_tri_0));
	//normalizedVertexPos.y = mod_input_vertex_centerOfRotation_pos.y + (dirToNewVertPos.y * someTester);
	//normalizedVertexPos.z = mod_input_vertex_centerOfRotation_pos.z + (dirToNewVertPos.z * someTester);
	
	//input.position.x = normalizedVertexPos.x;
	//input.position.y = normalizedVertexPos.y;
	//input.position.z = normalizedVertexPos.z;









	/*float differenceInAnglePitchForCurrent = AngleBetween(dir_ori_object.x,dir_ori_object.y,dir_ori_object_direction.x,dir_ori_object_direction.y);

	float differenceInAngleYawForCurrent = AngleBetween(dir_ori_object.z,dir_ori_object.x,dir_ori_object_direction.y,dir_ori_object_direction.x);

	float pointFrontX = (1 * cos(differenceInAnglePitchForCurrent)) + input.instancePosition.x;
	float pointFrontY = (1 * sin(differenceInAnglePitchForCurrent)) + input.instancePosition.y;
	float pointFrontZ = (1 * sin(differenceInAnglePitchForCurrent)) + input.instancePosition.z;

	float adjacent_tri_0 = cos(differenceInAnglePitchForCurrent) * hypothenuse; // x length
	float opposite_tri_0 = sin(differenceInAnglePitchForCurrent) * hypothenuse; // y length

	float3 normalizedVertexPos = float3(pointFrontX,pointFrontY,pointFrontZ);

	float3 dirToNewVertPos = normalizedVertexPos - mod_input_vertex_centerOfRotation_pos;

	normalizedVertexPos.x = normalizedVertexPos.x + (dirToNewVertPos.x * adjacent_tri_0);
	normalizedVertexPos.y = normalizedVertexPos.y + (dirToNewVertPos.y * opposite_tri_0);
	normalizedVertexPos.z = normalizedVertexPos.z + (dirToNewVertPos.z * hypothenuse);
	
	input.position.x = normalizedVertexPos.x;
	input.position.y = normalizedVertexPos.y;
	input.position.z = normalizedVertexPos.z;*/









	/*
	float2 dir_ori_object_direction = float2(1,0);
	float angle_pitch = AngleBetween(dir_ori_object_direction.x,dir_ori_object_direction.y,dir_ori_to_mod_vertex.x,dir_ori_to_mod_vertex.y);
	 
	dir_ori_object_direction = float2(1,0);
	float angle_yaw = AngleBetween(dir_ori_object_direction.y,dir_ori_object_direction.x,dir_ori_to_mod_vertex.z,dir_ori_to_mod_vertex.x);

	//float2 someTest = float2(dir_ori_to_mod_vertex.z,dir_ori_to_mod_vertex.x) -  

	float pointFrontX = (1 * cos(angle_pitch)) + input.instancePosition.x;
	float pointFrontY = (1 * sin(angle_pitch)) + input.instancePosition.y;
	float pointFrontZ = (1 * sin(angle_yaw)) + input.instancePosition.z;

	float adjacent_tri_0 = cos(angle_pitch) * hypothenuse; // x length
	float opposite_tri_0 = sin(angle_pitch) * hypothenuse; // y length

	float3 normalizedVertexPos = float3(pointFrontX,pointFrontY,pointFrontZ);

	float3 dirToNewVertPos = normalizedVertexPos - mod_input_vertex_centerOfRotation_pos;

	normalizedVertexPos.x = normalizedVertexPos.x + (dirToNewVertPos.x * adjacent_tri_0);
	normalizedVertexPos.y = normalizedVertexPos.y + (dirToNewVertPos.y * opposite_tri_0);
	normalizedVertexPos.z = normalizedVertexPos.z + (dirToNewVertPos.z * hypothenuse);
	
	input.position.x = normalizedVertexPos.x;
	input.position.y = normalizedVertexPos.y;
	input.position.z = normalizedVertexPos.z;*/






	/*float2 dir_ori_object_direction = float2(1,0);
	float4 dir_ori_to_mod_vertex = mod_input_vertex_pos - input.instancePosition;
	float anglePitchNoRotation0 = AngleBetween(dir_ori_object_direction.x,dir_ori_object_direction.y,dir_ori_to_mod_vertex.x,dir_ori_to_mod_vertex.y);

	float adjacent_tri_0 = cos(anglePitchNoRotation0) * hypothanus; // x length
	float opposite_tri_0 = sin(anglePitchNoRotation0) * hypothanus; // y length
	

	float2 dir_current_object_direction = float2(input.instanceRadRot.x,input.instanceRadRot.y);
	dir_ori_object_direction = float2(1,0);

	float pitch = AngleBetween(dir_current_object_direction.x,dir_current_object_direction.y,dir_ori_object_direction.x,dir_ori_object_direction.y);

	dir_current_object_direction = float2(input.instanceRadRot.x,input.instanceRadRot.z);	
	dir_ori_object_direction = float2(0,1);
	float yaw = AngleBetween(dir_current_object_direction.x,dir_current_object_direction.y,dir_ori_object_direction.x,dir_ori_object_direction.y);
	
	float pointFrontX = (1 * cos(pitch)) + input.instancePosition.x;
	float pointFrontY = (1 * sin(pitch)) + input.instancePosition.y;
	float pointFrontZ = (1 * sin(yaw)) + input.instancePosition.z;

	float3 normalizedVertexPos = float3(pointFrontX,pointFrontY,pointFrontZ);

	float3 dirToNewVertPos = normalizedVertexPos - mod_input_vertex_centerOfRotation_pos;

	normalizedVertexPos.x = normalizedVertexPos.x + (dirToNewVertPos.x * adjacent_tri_0*5);
	normalizedVertexPos.y = normalizedVertexPos.y + (dirToNewVertPos.y * opposite_tri_0*5);
	normalizedVertexPos.z = normalizedVertexPos.z + (dirToNewVertPos.z * hypothanus*5);

	input.position.x = normalizedVertexPos.x;
	input.position.y = normalizedVertexPos.y;
	input.position.z = normalizedVertexPos.z;*/



	















	//float4 dir_ori_to_mod_vertex = mod_input_vertex_pos - input.instancePosition;
	//float2 dir_parent_object = float2(input.instanceRadRot.x,input.instanceRadRot.y);
	//float2 dir_ori_object_direction = float2(world_right.x,world_right.y);
	//float anglePitchNoRotation0 = AngleBetween(dir_ori_object_direction.x,dir_ori_object_direction.y,dir_ori_to_mod_vertex.x,dir_ori_to_mod_vertex.y);

	//float pointFrontX = (1 * cos(anglePitchNoRotation0)) + input.instancePosition.x;
	//float pointFrontY = (1 * sin(anglePitchNoRotation0)) + input.instancePosition.y;
	//float pointFrontZ = (1 * tan(anglePitchNoRotation0)) + input.instancePosition.z;

	//input.position.x = pointFrontX;
	//input.position.y = pointFrontY;
	//input.position.z = pointFrontZ;














	//float2 dirObject0 = float2(input.instanceRadRot.x,input.instanceRadRot.y);
	//float2 dirWorld0 = float2(world_right.x,world_right.y);
	//float anglePitchNoRotation0 = AngleBetween(dirWorld0.x,dirWorld0.y,mod_input_vertex_pos.x,mod_input_vertex_pos.y);

	//float anglePitchNoRotation1 = AngleBetween(dirWorld0.x,dirWorld0.y,dirObject0.x,dirObject0.y);

	//anglePitchNoRotation1 += anglePitchNoRotation0;
	//anglePitchNoRotation1 = RadianToDegree(anglePitchNoRotation1);
	//anglePitchNoRotation1 = _normalize_degrees(anglePitchNoRotation1);
	//anglePitchNoRotation1 = DegreeToRadian(anglePitchNoRotation1);


	//float hypothanus = distance(float3(mod_input_vertex_pos.x,mod_input_vertex_pos.y,mod_input_vertex_pos.z), float3(mod_input_vertex_centerOfRotation_pos.x,mod_input_vertex_centerOfRotation_pos.y,mod_input_vertex_centerOfRotation_pos.z));
	//float adjacent_tri_0 = cos(anglePitchNoRotation1) * hypothanus; // x length
	//float opposite_tri_0 = sin(anglePitchNoRotation1) * hypothanus; // y length

	//float pointFrontX = (1 * cos(anglePitchNoRotation1)) + input.instancePosition.x;
	//float pointFrontY = (1 * sin(anglePitchNoRotation1)) + input.instancePosition.y;
	//float pointFrontZ = (1 * tan(anglePitchNoRotation1)) + input.instancePosition.z;

	//float3 normalizedVertexPos = float3(pointFrontX,pointFrontY,pointFrontZ);

	//normalizedVertexPos.x = normalizedVertexPos.x + (normalizedVertexPos.x * adjacent_tri_0);
	//normalizedVertexPos.y = normalizedVertexPos.y + (normalizedVertexPos.y * opposite_tri_0);
	//normalizedVertexPos.z = normalizedVertexPos.z + (normalizedVertexPos.z * hypothanus);
	

	//input.position.x = normalizedVertexPos.x;
	//input.position.y = normalizedVertexPos.y;
	//input.position.z = normalizedVertexPos.z;
	//input.position.z = mod_input_vertex_pos.z;










	//float2 dirObject1 = float2(input.instanceRadRot.x,input.instanceRadRot.z);
	//float2 dirWorld1 = float2(world_forward.x,world_forward.z);
	//float angleYawNoRotation = AngleBetween(dirWorld1.x,dirWorld1.y,dirWorld1.x,dirWorld1.y);

	//float2 dirObject2 = float2(input.instanceRadRot.y,input.instanceRadRot.z);
	//float2 dirWorld2 = float2(world_forward.y,world_forward.z);
	//float angleRollNoRotation = AngleBetween(dirWorld2.x,dirWorld2.y,dirObject2.x,dirObject2.y);


	
	//dirWorld0 = float2(world_forward.x,world_forward.z);
	//float angleYaw = AngleBetween(dirWorld0.x,dirWorld0.y,input.instanceRadRot.x,input.instanceRadRot.z);
	//SC_RotateVector2d();

	//float hypothanus = distance(float3(mod_input_vertex_pos.x,mod_input_vertex_pos.y,mod_input_vertex_pos.z), float3(mod_input_vertex_centerOfRotation_pos.x,mod_input_vertex_centerOfRotation_pos.y,mod_input_vertex_centerOfRotation_pos.z));
	//float adjacent_tri_0 = cos(anglePitchNoRotation1) * hypothanus; // x length
	//float opposite_tri_0 = sin(anglePitchNoRotation1) * hypothanus; // y length
	
	
	
	//float pointFrontX = (1 * cos(anglePitchNoRotation1)) + input.instancePosition.x;
	//float pointFrontY = (1 * sin(anglePitchNoRotation1)) + input.instancePosition.y;
	//float pointFrontZ = (1 * tan(anglePitchNoRotation1)) + input.instancePosition.z;

	//float3 normalizedVertexPos = float3(pointFrontX,pointFrontY,pointFrontZ);

	//normalizedVertexPos.x = normalizedVertexPos.x + (normalizedVertexPos.x * hypothanus);
	//normalizedVertexPos.y = normalizedVertexPos.y + (normalizedVertexPos.y * hypothanus);
	//normalizedVertexPos.z = normalizedVertexPos.z + (normalizedVertexPos.z * hypothanus);


	//input.position.x = normalizedVertexPos.x;
	//input.position.y = normalizedVertexPos.y;
	//input.position.z = normalizedVertexPos.z;
	//input.position.z = mod_input_vertex_pos.z;


	 //float2 dirObject1 = float2(input.instanceRadRot.x,input.instanceRadRot.y);
	 //float2 dirWorld1 = float2(world_right.x,world_right.y);
	 //float angleYaw = AngleBetween(dirObject1.x,dirObject1.y,dirWorld1.x,dirWorld1.y);
	 //float pointFrontY = (1 * sin(input.instanceRadRot.x)) + mod_input_vertex_centerOfRotation_pos.y;
















	//float4 dir_ori_to_mod_vertex = mod_input_vertex_pos - mod_input_vertex_centerOfRotation_pos;
	//float hypothanus = distance(float3(mod_input_vertex_pos.x,mod_input_vertex_pos.y,mod_input_vertex_pos.z), float3(mod_input_vertex_centerOfRotation_pos.x,mod_input_vertex_centerOfRotation_pos.y,mod_input_vertex_centerOfRotation_pos.z));


	//float hypothanus = length(dir_ori_to_mod_vertex); 
	//float adjacent_tri_0 = cos(input.instanceRadRot.x) * hypothanus; // x length
	//float opposite_tri_0 = sin(input.instanceRadRot.x) * hypothanus; // y length

	//float pointFrontX = (1 * cos(input.instanceRadRot.x)) + mod_input_vertex_centerOfRotation_pos.x;
    ///float pointFrontY = (1 * sin(input.instanceRadRot.x)) + mod_input_vertex_centerOfRotation_pos.y;
	//float pointFrontZ = (1 * tan(input.instanceRadRot.x)) + mod_input_vertex_centerOfRotation_pos.z;
	//float zcoord = (opposite_tri_0/adjacent_tri_0) + mod_input_vertex_centerOfRotation_pos.z;

	//float2 dirToPoint0 = float2(pointFrontX,pointFrontY) - float2(mod_input_vertex_centerOfRotation_pos.x,mod_input_vertex_centerOfRotation_pos.y);
	//float2 dirToPoint1 = float2(pointFrontZ,pointFrontY) - float2(mod_input_vertex_centerOfRotation_pos.z,mod_input_vertex_centerOfRotation_pos.y);

	//pointFrontX = pointFrontX + (dirToPoint0 * adjacent_tri_0);
	//pointFrontY = pointFrontY + (dirToPoint0 * opposite_tri_0);
	//pointFrontZ = pointFrontZ + (dirToPoint1 * hypothanus);



	//pointFrontX = pointFrontX;
	//pointFrontY = pointFrontY;
	//pointFrontZ = pointFrontZ;

	//input.position.x = mod_input_vertex_pos.x + input.instancePosition.x;
	//input.position.y = pointFrontY + input.instancePosition.y;
	//input.position.z = mod_input_vertex_pos.z + input.instancePosition.z;

	//input.position.x = mod_input_vertex_pos.x;
	//input.position.y = pointFrontY;
	//input.position.z = mod_input_vertex_pos.z;









	//float test0 =  input.instanceRadRot.x + (mod_input_vertex_pos.x)


	//float2 instanceCenterPOSRot0 = float2(input.instancePosition.x,input.instancePosition.y);
	//float2 somePoint0 =  float2(mod_input_vertex_pos.x,mod_input_vertex_pos.y);

	//float cosTheta = cos(input.instanceRadRot.z); //x
    //float sinTheta = sin(input.instanceRadRot.z); //y
    //float newX = (cosTheta * (somePoint0.x - instanceCenterPOSRot0.x) - sinTheta * (somePoint0.y - instanceCenterPOSRot0.y) + instanceCenterPOSRot0.x);
    //float newY = (sinTheta * (somePoint0.x - instanceCenterPOSRot0.x) + cosTheta * (somePoint0.y - instanceCenterPOSRot0.y) + instanceCenterPOSRot0.y);
	//mod_input_vertex_pos = float4(newX,mod_input_vertex_pos.y,mod_input_vertex_pos.z,input.position.w);

	//input.position = mod_input_vertex_pos;

	//instanceCenterPOSRot0 = float2(input.instancePosition.x,input.instancePosition.y);
	//somePoint0 =  float2(mod_input_vertex_pos.x,mod_input_vertex_pos.y);

	//cosTheta = cos(input.instanceRadRot.z); //x
    //sinTheta = sin(input.instanceRadRot.z); //y
    //newX = (cosTheta * (newX - instanceCenterPOSRot0.x) - sinTheta * (somePoint0.y - instanceCenterPOSRot0.y) + instanceCenterPOSRot0.x);
    //newY = (sinTheta * (newX - instanceCenterPOSRot0.x) + cosTheta * (somePoint0.y - instanceCenterPOSRot0.y) + instanceCenterPOSRot0.y);
	//mod_input_vertex_pos = float4(mod_input_vertex_pos.x,newY,mod_input_vertex_pos.z,input.position.w);
	
	



















	//float4 dir_ori_to_mod_vertex = mod_input_vertex_pos - mod_input_vertex_centerOfRotation_pos;
	//float hypothanus = length(dir_ori_to_mod_vertex); 
	//float adjacent_tri_0 = cos(input.instanceRadRot.z) * hypothanus; // x length
	//float opposite_tri_0 = sin(input.instanceRadRot.z) * hypothanus; // y length


	//dot(); or cross(); ?? 

	//input.position.x = input.position.x * input.instanceRadRot.x;
	//input.position.y = input.position.y * input.instanceRadRot.y;
	//input.position.z = input.position.z * input.instanceRadRot.z;



























	//float2 instanceCenterPOSRot0 = float2(input.instancePosition.x,input.instancePosition.y);
	//float2 somePoint0 =  float2(mod_input_vertex_pos.x,mod_input_vertex_pos.y);

	//float cosTheta = cos(input.instanceRadRot.z); //x
    //float sinTheta = sin(input.instanceRadRot.z); //y
    //float newX = (cosTheta * (somePoint0.x - instanceCenterPOSRot0.x) - sinTheta * (somePoint0.y - instanceCenterPOSRot0.y) + instanceCenterPOSRot0.x);
    //float newY = (sinTheta * (somePoint0.x - instanceCenterPOSRot0.x) + cosTheta * (somePoint0.y - instanceCenterPOSRot0.y) + instanceCenterPOSRot0.y);
	//input.position = float4(newX,newY,mod_input_vertex_pos.z,input.position.w);

	//float2 rotatedPointOne = SC_RotateVector2d(somePoint0, instanceCenterPOSRot0, input.instanceRadRot.x);

	//float2 instanceCenterPOSRot1 = float2(input.instancePosition.x,input.instancePosition.z);
	//somePoint1 =  float2(mod_input_vertex_pos.x, mod_input_vertex_pos.z);
	//rotatedPointOne = SC_RotateVector2d(somePoint1, instanceCenterPOSRot1, input.instanceRadRot.y);
	//input.position = float4(mod_input_vertex_pos.x,rotatedPointOne.y,rotatedPointOne.x,input.position.w);

	//float2 instanceCenterPOSRot2 = float2(input.instancePosition.x,input.instancePosition.y);
	//somePoint2 =  float2(mod_input_vertex_pos.x,mod_input_vertex_pos.y);
	//rotatedPointOne = SC_RotateVector2d(somePoint2, instanceCenterPOSRot2, input.instanceRadRot.z);
	//input.position = float4(rotatedPointOne.x,rotatedPointOne.y,mod_input_vertex_pos.z,mod_input_vertex_pos.w);


	/*float2 instanceCenterPOSRot0 = float2(input.instancePosition.x,input.instancePosition.y);
	float2 somePoint0 =  float2(mod_input_vertex_pos.x, mod_input_vertex_pos.y);

	float cosTheta = cos(input.instanceRadRot.z);
    float sinTheta = sin(input.instanceRadRot.z);	
	float newX = (cosTheta * (somePoint0.x - instanceCenterPOSRot0.x) - sinTheta * (somePoint0.y - instanceCenterPOSRot0.y) + instanceCenterPOSRot0.x);
    float newY = (sinTheta * (somePoint0.x - instanceCenterPOSRot0.x) + cosTheta * (somePoint0.y - instanceCenterPOSRot0.y) + instanceCenterPOSRot0.y);

	mod_input_vertex_pos.x =  newX;
	mod_input_vertex_pos.y =  newY;

	input.position = mod_input_vertex_pos.x;
    input.position = mod_input_vertex_pos.y;
    input.position = mod_input_vertex_pos.z;*/


	//input.position.x += input.instancePosition.x;
	//input.position.y += input.instancePosition.y;
	//input.position.z += input.instancePosition.z;

	//cosTheta = cos(input.instanceRadRot.z);
	//sinTheta = sin(input.instanceRadRot.z);	
	//newX = (cosTheta * (somePoint0.x - instanceCenterPOSRot0.x) - sinTheta * (somePoint0.y - instanceCenterPOSRot0.y) + instanceCenterPOSRot0.x);
	//newY = (sinTheta * (somePoint0.x - instanceCenterPOSRot0.x) + cosTheta * (somePoint0.y - instanceCenterPOSRot0.y) + instanceCenterPOSRot0.y);

	//mod_input_vertex_pos.y =  newY;

	//SC_RotateVector2d(somePoint0,instanceCenterPOSRot0,);

	//float2 instanceCenterPOSRot0 = float2(input.instancePosition.x,input.instancePosition.y);
	//float2 somePoint0 =  float2(mod_input_vertex_pos.x, mod_input_vertex_pos.y);
	
	//float cosTheta = cos(angleInRadians);
    //float sinTheta = sin(angleInRadians);	














	/*//https://www.raspberrypi.org/forums/viewtopic.php?t=109069
	//yaw y
	mod_input_vertex_pos.x = mod_input_vertex_pos.x * cos(input.instanceRadRot.y) - mod_input_vertex_pos.z * sin(input.instanceRadRot.y);
	mod_input_vertex_pos.z = mod_input_vertex_pos.x * sin(input.instanceRadRot.y) + mod_input_vertex_pos.z * cos(input.instanceRadRot.y);

	//pitch x
	mod_input_vertex_pos.y = mod_input_vertex_pos.y * cos( input.instanceRadRot.x) - mod_input_vertex_pos.z * sin( input.instanceRadRot.x);
	mod_input_vertex_pos.z = mod_input_vertex_pos.y * sin( input.instanceRadRot.x) + mod_input_vertex_pos.z * cos( input.instanceRadRot.x);

	//roll z
	mod_input_vertex_pos.x = mod_input_vertex_pos.x * cos(input.instanceRadRot.z) - mod_input_vertex_pos.y * sin(input.instanceRadRot.z);
	mod_input_vertex_pos.y = mod_input_vertex_pos.x * sin(input.instanceRadRot.z) + mod_input_vertex_pos.y * cos(input.instanceRadRot.z);

	input.position.x = mod_input_vertex_pos.x;
	input.position.y = mod_input_vertex_pos.y;
	input.position.z = mod_input_vertex_pos.z;*/