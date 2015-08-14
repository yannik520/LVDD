#include <libavcodec/avcodec.h>  
#include <libavformat/avformat.h>  
#include <libswscale/swscale.h>  

#include <stdio.h>  

#define DEBUG
#ifdef DEBUG
#define debug(format, arg...)    fprintf(stderr, "%s#%i: "format, __func__, __LINE__, ##arg)
#else
#define debug(format, arg...)    do{} while(0)
#endif


static struct SwsContext *img_convert_ctx;  

void SaveFrame(AVFrame *pFrame, int width, int height, int iFrame)  
{  
	FILE  *pFile;  
	char   szFilename[32];  
	int    y;  

	sprintf(szFilename, "frame%d.ppm", iFrame);  
	pFile = fopen(szFilename, "wb");  
	if (pFile == NULL) 
	{  
		return;  
	}  
	
	fprintf(pFile, "P6\n%d %d\n255\n", width, height);  
	
	for (y = 0; y < height; y++)
	{  
		fwrite(pFrame->data[0]+y*pFrame->linesize[0], 1, width*3, pFile);  
	}  
	
	fclose(pFile);  
}

int main(int argc, char *argv[])  
{  
	AVFormatContext  *pFormatCtx;  
	int       i, videoStream;  
	AVCodecContext   *	pCodecCtx;  
	AVCodec        *	pCodec;  
	AVFrame        *	pFrame;  
	AVFrame        *	pFrameRGB;  
	AVPacket      	packet;  
	int       		frameFinished;  
	int            	numBytes;  
	uint8_t        *buffer;

	if (argc < 2) 
	{  
		printf("Please provide a movie file\n");  
		return -1;  
	}  

	debug("ARG OK.\n");  

	avcodec_register_all();
	av_register_all();
	avformat_network_init();
	
	debug("av_register_all() OK.\n");  

	pFormatCtx = avformat_alloc_context(); // ·ÖÅä¿Õ¼ä  
	debug("avformat_alloc_context() OK.\n");  
	if (avformat_open_input(&pFormatCtx, argv[1], NULL, NULL) != 0) 
	{  
		return -1;  
	}  
	
	debug("avformat_open_input() OK.\n");  

	if (avformat_find_stream_info(pFormatCtx, NULL) < 0) 
	{  
		return -1;  
	}
	debug("av_find_stream_info() OK.\n");  

	av_dump_format(pFormatCtx, 0, argv[1], 0);  
	debug("dump_format() OK.\n");  

	videoStream = -1;  
	for(i=0; i<pFormatCtx->nb_streams; i++) 
	{  
		if (pFormatCtx->streams[i]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {  
			videoStream = i;  
			break;  
		}  
	}  
	debug("find CODEC_TYPE_VIDEO OK.\n");  

	if(videoStream == -1) 
	{  
		return -1;  
	}  

	pCodecCtx=pFormatCtx->streams[videoStream]->codec;  
	img_convert_ctx = sws_getContext(  
		pCodecCtx->width,   
		pCodecCtx->height,  
		pCodecCtx->pix_fmt,   
		pCodecCtx->width,   
		pCodecCtx->height,   
		PIX_FMT_RGB24,   
		SWS_BICUBIC,   
		NULL, NULL, NULL);  
	debug("sws_getContext() OK.\n");  

	pCodec = avcodec_find_decoder(pCodecCtx->codec_id);  
	if (pCodec == NULL) 
	{  
		fprintf(stderr, "Unsupported codec!\n");  
		return -1;  
	}  
	debug("avcodec_find_decoder() OK.\n");  

	if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {  
		return -1;  
	}  
	debug("avcodec_open() OK.\n");  

	pFrame = avcodec_alloc_frame();  
	debug("avcodec_alloc_frame() OK.\n");  

	pFrameRGB = avcodec_alloc_frame();  
	if (pFrameRGB == NULL) {  
		return -1;  
	}  
	debug("avcodec_alloc_frame() OK.\n");  

	numBytes = avpicture_get_size(PIX_FMT_RGB24, pCodecCtx->width,  
		pCodecCtx->height);  
	debug("avpicture_get_size() OK.\n");  

	buffer = (uint8_t *)av_malloc(numBytes*sizeof(uint8_t));  

	avpicture_fill((AVPicture *)pFrameRGB, buffer, PIX_FMT_RGB24,  
		pCodecCtx->width, pCodecCtx->height);  
	debug("avpicture_fill() OK.\n");  
	i = 0;  
	while(av_read_frame(pFormatCtx, &packet) >= 0) 
	{  
		if (packet.stream_index == videoStream) 
		{  
			avcodec_decode_video2(pCodecCtx, pFrame,   
				&frameFinished,  
				&packet);  
			if(frameFinished) 
			{  
				sws_scale(img_convert_ctx,   
					(const uint8_t **)pFrame->data,   
					pFrame->linesize, 0,   
					pCodecCtx->height,   
					pFrameRGB->data,   
					pFrameRGB->linesize);  
				if (++i <= 1000) 
				{  
					SaveFrame(pFrameRGB, pCodecCtx->width,pCodecCtx->height,i);  
				}  
			}  
		}  

		av_free_packet(&packet);  
	}  

	av_free(buffer);  
	av_free(pFrameRGB);  

	av_free(pFrame);  

	avcodec_close(pCodecCtx);  

	avformat_close_input(&pFormatCtx);  

	debug("End.\n");

	return 0;  
} 

