#include <stdio.h>
#include <stdlib.h>
#include <alsa/asoundlib.h>
#include "/opt/ros/hydro/include/ros/ros.h"
#include "audio_common_msgs/AudioData.h"


int buf_size = 448;
short lbuf[448];

void chatterCallback(audio_common_msgs::AudioData msg)
{
	for(int i =0; i<buf_size; i++)
	{
	//	std::cout<<msg.data[i];
		lbuf[i] = msg.data[i];
	}
	//memcpy( &lbuf[0], &msg.data, buf_size);
	//printf("copied msg data to buffer\n");
}




int main (int argc, char *argv[])
{
	int i;
	int err;
	short buf[448];
	snd_pcm_t *playback_handle;
	snd_pcm_hw_params_t *hw_params;
	unsigned int sample_rate = 44100;

	ros::init(argc, argv, "playback");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("audio_publisher", 1000, chatterCallback);

	ros::spinOnce();


	if ((err = snd_pcm_open (&playback_handle, argv[1], SND_PCM_STREAM_PLAYBACK, 0)) < 0) {
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

	if ((err = snd_pcm_hw_params_any (playback_handle, hw_params)) < 0) {
		fprintf (stderr, "cannot initialize hardware parameter structure (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params_set_access (playback_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
		fprintf (stderr, "cannot set access type (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params_set_format (playback_handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
		fprintf (stderr, "cannot set sample format (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params_set_rate_near (playback_handle, hw_params, &sample_rate, 0)) < 0) {
		fprintf (stderr, "cannot set sample rate (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params_set_channels (playback_handle, hw_params, 2)) < 0) {
		fprintf (stderr, "cannot set channel count (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	if ((err = snd_pcm_hw_params (playback_handle, hw_params)) < 0) {
		fprintf (stderr, "cannot set parameters (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	snd_pcm_hw_params_free (hw_params);

	if ((err = snd_pcm_prepare (playback_handle)) < 0) {
		fprintf (stderr, "cannot prepare audio interface for use (%s)\n",
				snd_strerror (err));
		exit (1);
	}

	while(ros::ok()){

		for(int i =0; i<buf_size; i++)
			{
			//	std::cout<<msg.data[i];
				buf[i] = lbuf[i];
			}
		if ((err = snd_pcm_writei (playback_handle, buf, buf_size)) != buf_size) {
			fprintf (stderr, "write to audio interface failed (%s)\n",
					snd_strerror (err));
			exit (1);
		}

		//writeWAVData("tester.wav", lbuf, 32, 22050, 7);


		ros::spinOnce();

	}

	snd_pcm_close (playback_handle);
	exit (0);
}



