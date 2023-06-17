#ifndef DATA_STRUCT_H
#define DATA_STRUCT_H

typedef struct
{
    unsigned char	road_type:4;
    unsigned char	right_wall:1;
    unsigned char	left_wall:1;
    unsigned char	road_direction:2;
    unsigned char	reserve:8;
} Property;
typedef union
{
    Property data;
    char byte[sizeof(Property)];
}PropertyUnion;

typedef struct _GPSInfo
{
  unsigned short int road_id;
  float iHead;//方位角
  double iLatitude;//纬度
  double iLongitude;//经度
  float altitude;//高度
  float velocity;
  float left_width;
  float right_width;
  PropertyUnion property;
  float iDistance;//距离
  float iCurvature;//曲率
  float dist_origin;//距离原点的距离
}GPSInfo;


typedef struct _ENInfo
{
  bool    iValid;          //有效性
  bool    iOnLine;         //在线判定
  bool    reverse;         //是否为倒车
  double  iHead;           //航向
  double  x;              //纬度
  double  y;              //经度
  double  z;              //高程
  double xEN;            //东北天坐标系x
  double yEN;           //东北天坐标系y
  double zEN;           //东北天坐标系z
  double iDistance;        //距离
  double iCurvature;       //曲率
  double P_radius;         //卯酉半径
  double theta;            //与东偏向
}ENInfo;

#endif // DATA_STRUCT_H
