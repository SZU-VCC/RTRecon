#include "FrameRecon.h"
#include "OutputUtils.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <pcl/io/ply_io.h>
#include <thread>
#include <random>

template<class T>
void FrameRecon::PublishPointCloud(pcl::PointCloud<T>& pointcloud, ros::Publisher& publisher)
{
    sensor_msgs::PointCloud2 vCloudData;
	pcl::toROSMsg(pointcloud, vCloudData);
	vCloudData.header.frame_id = odomFrameId;
	vCloudData.header.stamp = ros::Time::now();
	publisher.publish(vCloudData);
}
template void FrameRecon::PublishPointCloud(pcl::PointCloud<pcl::PointXYZI>& pointcloud, ros::Publisher& publisher);
template void FrameRecon::PublishPointCloud(pcl::PointCloud<pcl::PointNormal>& pointcloud, ros::Publisher& publisher);



void FrameRecon::PublishFramePointsNormal(const pcl::PointCloud<pcl::PointNormal> & vCloudNormal){
    sensor_msgs::PointCloud2 vCloudData;
	pcl::toROSMsg(vCloudNormal, vCloudData);
	vCloudData.header.frame_id =odomFrameId;
	vCloudData.header.stamp = ros::Time::now();
	FramePointsNormalPublisher.publish(vCloudData);
}

void FrameRecon::PublishAdditionalPoints(const pcl::PointCloud<pcl::PointNormal> & vCloudNormal){
    sensor_msgs::PointCloud2 vCloudData;
	pcl::toROSMsg(vCloudNormal, vCloudData);
	vCloudData.header.frame_id = odomFrameId;
	vCloudData.header.stamp = ros::Time::now();
	AdditionalPointsPublisher.publish(vCloudData);
}


void FrameRecon::PublishFrameMesh(){
  	
  	//new a visual message
	visualization_msgs::Marker oMeshMsgs;
	
	//define header of message
	oMeshMsgs.header.frame_id = odomFrameId;
	oMeshMsgs.header.stamp = ros::Time::now();
	oMeshMsgs.type = visualization_msgs::Marker::TRIANGLE_LIST;
	oMeshMsgs.action = visualization_msgs::Marker::ADD;

	oMeshMsgs.scale.x = 1.0;
	oMeshMsgs.scale.y = 1.0;
	oMeshMsgs.scale.z = 1.0;

	oMeshMsgs.pose.position.x = 0.0;
	oMeshMsgs.pose.position.y = 0.0;
	oMeshMsgs.pose.position.z = 0.0;

	oMeshMsgs.pose.orientation.x = 0.0;
	oMeshMsgs.pose.orientation.y = 0.0;
	oMeshMsgs.pose.orientation.z = 0.0;
	oMeshMsgs.pose.orientation.w = 1.0;

	std_msgs::ColorRGBA color;
	color.a = 1.0;
	// color.r = 200 / 255.f * 1.5f;
	// color.g = 128 / 255.f * 1.5f;
	// color.b = 54  / 255.f * 1.5f;
	
	color.r = 1.0;
	color.g = 1.0;
	color.b = 1.0;

	//repeatable vertices
	pcl::PointCloud<pcl::PointXYZI> vMeshVertices;

	//get the reconstruted mesh
	meshAlgoBuilder.OutputAllShowMeshes(vMeshVertices);

	//convert to publishable message
	for (int k = 0; k < vMeshVertices.points.size(); ++k){

		//temp point
    	geometry_msgs::Point oPTemp;
        oPTemp.x = vMeshVertices.points[k].x;
        oPTemp.y = vMeshVertices.points[k].y;
        oPTemp.z = vMeshVertices.points[k].z;

        //color
        oMeshMsgs.points.push_back(oPTemp);
        oMeshMsgs.color = color;

	}//end k

	FrameMeshPublisher.publish(oMeshMsgs);
}

void FrameRecon::PublishFrameMeshAlgorithm() {

	fusion_msgs::MeshArray oMeshArray;

	for(int i = 0; i < meshAlgoBuilder.m_vAllSectorClouds.size(); ++i) {

		shape_msgs::Mesh mesh;
		meshAlgoBuilder.OutputSectorMesh(mesh, i);
		oMeshArray.data.push_back(mesh);

		fusion_msgs::TokenArray arr;
		arr.tokens = meshAlgoBuilder.m_vFaceWeight[i];
		oMeshArray.pseudo_tokens.push_back(arr);
	}

	FrameMeshAlgoPublisher.publish(oMeshArray);
}

void FrameRecon::PublishFrameMeshAlgorithmView() {

	  	//new a visual message
	visualization_msgs::Marker oMeshMsgs;
	
	//define header of message
	oMeshMsgs.header.frame_id = odomFrameId;
	oMeshMsgs.header.stamp = ros::Time::now();
	oMeshMsgs.type = visualization_msgs::Marker::TRIANGLE_LIST;
	oMeshMsgs.action = visualization_msgs::Marker::ADD;

	oMeshMsgs.scale.x = 1.0;
	oMeshMsgs.scale.y = 1.0;
	oMeshMsgs.scale.z = 1.0;

	oMeshMsgs.pose.position.x = 0.0;
	oMeshMsgs.pose.position.y = 0.0;
	oMeshMsgs.pose.position.z = 0.0;

	oMeshMsgs.pose.orientation.x = 0.0;
	oMeshMsgs.pose.orientation.y = 0.0;
	oMeshMsgs.pose.orientation.z = 0.0;
	oMeshMsgs.pose.orientation.w = 1.0;

	std_msgs::ColorRGBA color;
	color.a = 1.0f;
	color.r = 0.2f;
	color.g = 1.0f;
	color.b = 1.0f;
	oMeshMsgs.color = color;

	//repeatable vertices
	pcl::PointCloud<pcl::PointXYZI> vMeshVertices;

	//get the reconstruted mesh
	meshAlgoBuilder.OutputAllMeshes(vMeshVertices);

	//convert to publishable message
	for (int k = 0; k < vMeshVertices.points.size(); ++k){

		//temp point
    	geometry_msgs::Point oPTemp;
        oPTemp.x = vMeshVertices.points[k].x;
        oPTemp.y = vMeshVertices.points[k].y;
        oPTemp.z = vMeshVertices.points[k].z;

        //color
		// if(vMeshVertices.points[k].intensity == 0) continue;
		std_msgs::ColorRGBA color_temp = color;
		color_temp.b = clamp(vMeshVertices.points[k].intensity, 0.f, 0.1f) * 10;
		color_temp.g = clamp(0.1f - vMeshVertices.points[k].intensity, 0.f, 0.1f) * 10;
		oMeshMsgs.colors.push_back(color_temp);
		oMeshMsgs.points.push_back(oPTemp);

	}//end k

	FrameMeshAlgoViewPublisher.publish(oMeshMsgs);
}
