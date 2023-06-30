#pragma once
#include <tf2/LinearMath/Vector3.h>


static float signedAngle(const tf2::Vector3& v1, const tf2::Vector3& v2, const tf2::Vector3& referenceAxis)
{
    float sign = (v1.cross(v2).dot(referenceAxis)) >0? 1 : -1;
    return v1.angle(v2) * sign;
}

static float signedDistanceToLine(const tf2::Vector3& lineOrigin, const tf2::Vector3& lineDirection, const tf2::Vector3& p )
{
    tf2::Vector3 vecToP = p-lineOrigin;
    tf2::Vector3 projected = lineOrigin + lineDirection.normalized() * lineDirection.normalized().dot(vecToP);
    
    float sign = signedAngle( lineDirection, vecToP, {0,0,1}) >0? 1:-1;

    return tf2::tf2Distance2(p, projected) * sign;
}