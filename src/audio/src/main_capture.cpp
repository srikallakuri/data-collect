#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>
#include <ros/ros.h>
#include "audio_common_msgs/AudioData.h"


main (int argc, char *argv[])
{

	ros::init(argc, argv, "capture");

	ros::NodeHandle _nh;
	ros::Publisher _pub = _nh.advertise<audio_common_msgs::AudioData>("audio_publisher", 1000, true);

	ros::Rate loop_rate(1000);


	int i;
	int err;
	unsigned int sample_rate=22050;
	short buf[128];
	snd_pcm_t *capture_handle;
	snd_pcm_hw_params_t *hw_params;


	int num_channels = 7;

	if(argc > 2)
		num_channels = atoi(argv[2]);



	if ((err = snd_pcm_open (&capture_handle, argv[1], SND_PCM_STREAM_CAPTURE, 0)) < 0) {
		fprintf (stderr, "cannot open audio device %s (%s)\n",
				argv[1],
				snd_strerror (err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params_malloc (&hw_params)) < 0) {
		fprintf (stderr, "cannot allocate hardware parameter structure (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params_any (capture_handle, hw_params)) < 0) {
		fprintf (stderr, "cannot initialize hardware parameter structure (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params_set_access (capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
		fprintf (stderr, "cannot set access type (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params_set_format (capture_handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
		fprintf (stderr, "cannot set sample format (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	//printf("came here \n");

	if ((err = snd_pcm_hw_params_set_rate_near (capture_handle, hw_params,&sample_rate, 0)) < 0) {
		fprintf (stderr, "cannot set sample rate (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	//printf("came here 3 \n");


	if ((err = snd_pcm_hw_params_set_channels (capture_handle, hw_params, num_channels)) < 0) {
		fprintf (stderr, "cannot set channel count (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params (capture_handle, hw_params)) < 0) {
		fprintf (stderr, "cannot set parameters (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	snd_pcm_hw_params_free (hw_params);

	if ((err = snd_pcm_prepare (capture_handle)) < 0) {
		fprintf (stderr, "cannot prepare audio interface for use (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	while(ros::ok()){
		if ((err = snd_pcm_readi (capture_handle, buf, 32)) != 32) {
			fprintf (stderr, "read from audio interface failed (%s)\n",
					snd_strerror (err));
			exit (1);
		}






		/////////////////////////////////////////////////////////////////////////
		/* put the ros message here */
		audio_common_msgs::AudioData msg;
		msg.data.resize( 32 );
		memcpy( &msg.data[0], buf, 32);
		//printf("came here \n");
		_pub.publish(msg);
		/////////////////////////////////////////////////////////////////////////
		printf("got the chunk of data\n");
		ros::spinOnce();
		loop_rate.sleep();

	}

	snd_pcm_close (capture_handle);
	exit (0);
}


