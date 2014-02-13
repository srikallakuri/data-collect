#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_msg.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <exception>
#include <iostream>
#include <fstream>
#include <DepthSense.hxx>
#include <opencv/highgui.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <sensor_msgs/image_encodings.h>
using namespace DepthSense;
using namespace std;

Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;

uint32_t g_aFrames = 0;
uint32_t g_cFrames = 0;
uint32_t g_dFrames = 0;

bool g_bDeviceFound = false;

ProjectionHelper* g_pProjHelper = NULL;
StereoCameraParameters g_scp;


/*----------------------------------------------------------------------------*/
// New color sample event handler
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
{

	depthsense_node::image_msg im_data;

	for(int i=0; i< 640*480; i++)
	{
		im_data.image_data.push_back((int) data.colorMap[i]);
	}
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<depthsense_node::image_msg>("camera/rgb/image_color", 1000);

	while (ros::ok())
	  {
	    pub.publish(im_data);

	    ros::spinOnce();	
	  }

	g_cFrames++;
}




/*----------------------------------------------------------------------------*/
// New depth sample event handler
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
	// Project some 3D points in the Color Frame
	if (!g_pProjHelper)
	{
		g_pProjHelper = new ProjectionHelper (data.stereoCameraParameters);
		g_scp = data.stereoCameraParameters;
	}
	else if (g_scp != data.stereoCameraParameters)
	{
		g_pProjHelper->setStereoCameraParameters(data.stereoCameraParameters);
		g_scp = data.stereoCameraParameters;
	}

	int32_t w, h;
	FrameFormat_toResolution(data.captureConfiguration.frameFormat,&w,&h);
	int cx = w/2;
	int cy = h/2;

	Vertex p3DPoints[4];

	p3DPoints[0] = data.vertices[(cy-h/4)*w+cx-w/4];
	p3DPoints[1] = data.vertices[(cy-h/4)*w+cx+w/4];
	p3DPoints[2] = data.vertices[(cy+h/4)*w+cx+w/4];
	p3DPoints[3] = data.vertices[(cy+h/4)*w+cx-w/4];

	Point2D p2DPoints[4];
	g_pProjHelper->get2DCoordinates ( p3DPoints, p2DPoints, 4, CAMERA_PLANE_COLOR);

	int size=data.vertices.size();

	//generate coordinate matrix
	int element_num_total=3*size;
	float *Ma=new float [element_num_total];



	int index_mat=0;
	for (int i=0;i<data.vertices.size();i++)
	{
		int index_0=i;
		int index_1=size+i;
		int index_2=2*size+i;
		Ma[index_0]=data.vertices[i].x;
		Ma[index_1]=data.vertices[i].y;
		Ma[index_2]=data.vertices[i].z;

	}

	//generate range matrix
	float *Ma_range=new float [size];
	float range_min=10000.0;
	float range_max=0.0;
	for (int i=0;i<size;i++)
	{
		float range=sqrt(data.vertices[i].x*data.vertices[i].x+data.vertices[i].y*data.vertices[i].y+data.vertices[i].z*data.vertices[i].z);
		Ma_range[i]=range;
		if(range>range_max)
		{
			range_max=range;
		}
		if(range<range_min)
		{
			range_min=range;
		}
	}

	//generate jpg for range value as depth image
	CvMat *Ma_range_jpg=cvCreateMat(240,320,CV_16UC3);
	for(int i=0;i<240;i++)
	{
		for(int j=0;j<320;j++)
		{
			CvScalar temp;
			int index_0=i*320+j;
			temp.val[0]=int((Ma_range[index_0]-range_min)*255/(range_max-range_min));
			temp.val[1]=int((Ma_range[index_0]-range_min)*255/(range_max-range_min));
			temp.val[2]=int((Ma_range[index_0]-range_min)*255/(range_max-range_min));
			cvSet2D(Ma_range_jpg,i,j,temp);

		}
	}

	std::stringstream ss;
	ss << g_dFrames;
	std::string index_str = ss.str();

	std::string depth_coordinate_path="/home/nkallaku/data/txt/depth_coordinate_"+index_str+".txt";
	string depth_range_path="/home/nkallaku/data/txt/depth_range_"+index_str+".txt";
	string depth_range_img_path="/home/nkallaku/data/img/depth/depth_img_"+index_str+".jpg";



	string output_string = "depth_coordinate_"+index_str+".txt"+" "+"depth_range_"+index_str+".txt"+" "+"depth_img_"+index_str+".jpg";

	int argc;
	char **argv;

	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<sensor_msgs::Image>("camera/depth/image", 1000);

	cv_bridge::CvImage out_msg;
	out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
	out_msg.image    = Ma_range_jpg; // Your cv::Mat

	//ros::Rate loop_rate(30);
	/*while (nh.ok())
	{
		if(pub.getNumSubscribers()>0)
		{
			pub.publish(out_msg.toImageMsg());

			break;
		}
	}*/

	//write Ma into txt file
	FILE *myfile;
	myfile=fopen(depth_coordinate_path.c_str(),"wb");
	fwrite(Ma, sizeof(float), element_num_total, myfile);
	fclose(myfile);

	//save Ma_range into range file
	FILE *range_file;
	range_file=fopen(depth_range_path.c_str(),"wb");
	fwrite(Ma_range,sizeof(float),size,range_file);
	fclose(range_file);

	//save Ma_range_img into file
	cvSaveImage(depth_range_img_path.c_str(),Ma_range_jpg);
	cvReleaseMat(&Ma_range_jpg);

	delete Ma;
	delete Ma_range;

	g_dFrames++;

	// Quit the main loop after 200 depth frames received
	if (g_dFrames == 10000)
		g_context.quit();
}

/*----------------------------------------------------------------------------*/
void configureDepthNode()
{
	g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);

	DepthNode::Configuration config = g_dnode.getConfiguration();
	config.frameFormat = FRAME_FORMAT_QVGA;
	config.framerate = 30;
	config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
	config.saturation = true;

	g_dnode.setEnableVertices(true);

	try
	{
		g_context.requestControl(g_dnode,0);

		g_dnode.setConfiguration(config);
	}
	catch (ArgumentException& e)
	{
		printf("Argument Exception: %s\n",e.what());
	}
	catch (UnauthorizedAccessException& e)
	{
		printf("Unauthorized Access Exception: %s\n",e.what());
	}
	catch (IOException& e)
	{
		printf("IO Exception: %s\n",e.what());
	}
	catch (InvalidOperationException& e)
	{
		printf("Invalid Operation Exception: %s\n",e.what());
	}
	catch (ConfigurationException& e)
	{
		printf("Configuration Exception: %s\n",e.what());
	}
	catch (StreamingException& e)
	{
		printf("Streaming Exception: %s\n",e.what());
	}
	catch (TimeoutException&)
	{
		printf("TimeoutException\n");
	}

}

/*----------------------------------------------------------------------------*/
void configureColorNode()
{
	// connect new color sample handler
	g_cnode.newSampleReceivedEvent().connect(&onNewColorSample);

	ColorNode::Configuration config = g_cnode.getConfiguration();
	config.frameFormat = FRAME_FORMAT_VGA;
	config.compression = COMPRESSION_TYPE_MJPEG;
	config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
	config.framerate = 30;

	g_cnode.setEnableColorMap(true);

	try
	{
		g_context.requestControl(g_cnode,0);

		g_cnode.setConfiguration(config);
	}
	catch (ArgumentException& e)
	{
		printf("Argument Exception: %s\n",e.what());
	}
	catch (UnauthorizedAccessException& e)
	{
		printf("Unauthorized Access Exception: %s\n",e.what());
	}
	catch (IOException& e)
	{
		printf("IO Exception: %s\n",e.what());
	}
	catch (InvalidOperationException& e)
	{
		printf("Invalid Operation Exception: %s\n",e.what());
	}
	catch (ConfigurationException& e)
	{
		printf("Configuration Exception: %s\n",e.what());
	}
	catch (StreamingException& e)
	{
		printf("Streaming Exception: %s\n",e.what());
	}
	catch (TimeoutException&)
	{
		printf("TimeoutException\n");
	}
}

/*----------------------------------------------------------------------------*/
void configureNode(Node node)
{
	if ((node.is<DepthNode>())&&(!g_dnode.isSet()))
	{
		g_dnode = node.as<DepthNode>();
		configureDepthNode();
		g_context.registerNode(node);

	}

	if ((node.is<ColorNode>())&&(!g_cnode.isSet()))
	{
		g_cnode = node.as<ColorNode>();
		configureColorNode();
		g_context.registerNode(node);
	}
}

/*----------------------------------------------------------------------------*/
void onNodeConnected(Device device, Device::NodeAddedData data)
{
	configureNode(data.node);
}

/*----------------------------------------------------------------------------*/
void onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
	if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode))
		g_cnode.unset();
	if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
		g_dnode.unset();
	//   printf("Node disconnected\n");
}

/*----------------------------------------------------------------------------*/
void onDeviceConnected(Context context, Context::DeviceAddedData data)
{
	if (!g_bDeviceFound)
	{
		data.device.nodeAddedEvent().connect(&onNodeConnected);
		data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
		g_bDeviceFound = true;
	}
}

/*----------------------------------------------------------------------------*/
void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
	g_bDeviceFound = false;
	//  printf("Device disconnected\n");
}


int main(int argc,char **argv)
{

	system("rm /home/nkallaku/data/img/color/color*");
	system("rm /home/nkallaku/data/img/depth/depth*");

	ros::init(argc, argv, "talker", true);

	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("image_publisher", 1000);

	//ros::Rate loop_rate(30);

	int count = 0;
	while (ros::ok())
	{
		std_msgs::String msg;

		std::stringstream ss;
		ss << "total data saved are in folder /home/aidas/collected_data/";
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());
		chatter_pub.publish(msg);

		g_context = Context::create("localhost");

		g_context.deviceAddedEvent().connect(&onDeviceConnected);
		g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);

		// Get the list of currently connected devices
		vector<Device> da = g_context.getDevices();

		// We are only interested in the first device
		if (da.size() >= 1)
		{
			g_bDeviceFound = true;

			da[0].nodeAddedEvent().connect(&onNodeConnected);
			da[0].nodeRemovedEvent().connect(&onNodeDisconnected);

			vector<Node> na = da[0].getNodes();

			printf("Found %u nodes\n",na.size());

			for (int n = 0; n < (int)na.size();n++)
			{
				configureNode(na[n]);

			}

		}

		g_context.startNodes();

		g_context.run();

		g_context.stopNodes();

		if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
		if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);

		if (g_pProjHelper)
			delete g_pProjHelper;

		ros::spinOnce();

	}


	return 0;
}

