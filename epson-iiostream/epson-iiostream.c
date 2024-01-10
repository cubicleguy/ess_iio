/*
 * Copyright (c) 2016, DAQRI. All rights reserved.
 * Author: Lucas Magasweran <lucas.magasweran@daqri.com>
 *
 * Based on AD9361 example:
 * Copyright (C) 2014 IABG mbH
 * Author: Michael Feilen <feilen_at_iabg.de>
 *
 * Copyright(C) SEIKO EPSON CORPORATION 2017-2022. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*
 * The epson-iiostream is based on the Dummy IIO streaming example.
 *
 * Version 1.20
 *
 * Command line examples to run epson-iiostream with local backend:
 * > sudo ./epson-iiostream
 * > sudo ./epson-iiostream -c 10
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <getopt.h>
#include <inttypes.h>
#include <iio.h>
#include <sys/time.h>


/* default names for device and trigger */
static char *device_name  =  "iio:device0";      /* device  name: epsonG354PDH0, iio:device0, ... */
static char *trigger_name =  "trigger0";         /* trigger name: epsonG354PDH0-dev0, trigger0, instance1, ... */
static int buffer_length  =  1;
static int count          = -1;
static char *sample_rate  = NULL;
static char *filter_sel   = NULL;

static double in_anglvel_scale;
static double in_tempX_scale;
static double in_accel_scale;
static double in_tempX_offset;


/* log file */
static FILE *fptr_log_file = NULL;
static char *log_file_name = NULL;
static int   logging_data  = false;

#define SZ_CH_SIZE   32

static char * get_temp ( int64_t rawdata, const struct iio_data_format * fmt );
static char * get_gyro ( int64_t rawdata, const struct iio_data_format * fmt );
static char * get_accel( int64_t rawdata, const struct iio_data_format * fmt );
static char * get_count( int64_t rawdata, const struct iio_data_format * fmt );
static char * get_time ( int64_t rawdata, const struct iio_data_format * fmt );


typedef struct {
        struct iio_channel * chn;
        const char attr_name[SZ_CH_SIZE];
        const char alt_name[SZ_CH_SIZE];
        const char * unit;
        char* (*fptr)( int64_t, const struct iio_data_format* );
        size_t enable;
} epson_chn;

#define CH_CNT            9
static epson_chn * epix[CH_CNT];

static char t_unit[SZ_CH_SIZE] = "deg.C";
static char g_unit[SZ_CH_SIZE] = "rad/s";
static char a_unit[SZ_CH_SIZE] = "m/s^2";
static char c_unit[SZ_CH_SIZE] = "dec";
static char s_unit[SZ_CH_SIZE] = "unix epoch time";

static epson_chn temp_ch      = { NULL, "temp",       "Ts",         t_unit, &get_temp,  true };
static epson_chn anglvel_ch_x = { NULL, "anglvel_x",  "Gx",         g_unit, &get_gyro,  true };
static epson_chn anglvel_ch_y = { NULL, "anglvel_y",  "Gy",         g_unit, &get_gyro,  true };
static epson_chn anglvel_ch_z = { NULL, "anglvel_z",  "Gz",         g_unit, &get_gyro,  true };
static epson_chn accel_ch_x   = { NULL, "accel_x",    "Ax",         a_unit, &get_accel, true };
static epson_chn accel_ch_y   = { NULL, "accel_y",    "Ay",         a_unit, &get_accel, true };
static epson_chn accel_ch_z   = { NULL, "accel_z",    "Az",         a_unit, &get_accel, true };
static epson_chn count_ch     = { NULL, "count",      "Counter",    c_unit, &get_count, true };
static epson_chn timestamp_ch = { NULL, "timestamp",  "Timestamp",  s_unit, &get_time,  true };

/* Streaming devices */
static struct iio_device *dev;

/* IIO structs required for streaming */
static struct iio_context *ctx;
static struct iio_buffer  *rxbuf;
static unsigned int channel_count;

static char product_id[16];
static char firmware_version[16];
static char serial_number[16];

static int64_t log_start_time = 0;
static int64_t log_stop_time = 0;

static bool stop;
static bool has_repeat;

#define STRTMP_SIZE  32
static char strtmp[STRTMP_SIZE];
static char * remote_host = NULL;


char * get_temp ( int64_t rawdata, const struct iio_data_format * fmt )
{
        /*
         * For 32-bit usage: T [deg.C]= (SF/65536) * ( A -172621824 ) + 25
         * For 16-bit usage: T [deg.C]= SF * ( A -2634 ) + 25
         */

        double A = (double)rawdata;
        double scale = fmt->scale / 1000.0f;
        double offset = in_tempX_offset;

        if ( fmt->length == 32 ) {
                scale  /= 65536.0f;
                offset *= 65536.0f;
        }
        A += offset;//< in_tempX_offset;
        A *= scale; //< in_tempX_scale;
        A += 25.0f;
        sprintf( strtmp, "%+.8f", A );

        return strtmp;
}

char * get_gyro ( int64_t rawdata, const struct iio_data_format * fmt )
{
        /*
         * For 32-bit usage: G [deg/s]= (SF/65536) * B
         * For 16-bit usage: G [deg/s]= SF * B
         */

        double G = (double)rawdata;
        double scale = fmt->scale; //< in_anglvel_scale

        if ( fmt->length == 32 ) {
                scale /= 65536.0f;
        }
        G *= scale;

        sprintf( strtmp, "%+.8f", G );

        return strtmp;
}

char * get_accel( int64_t rawdata, const struct iio_data_format * fmt )
{
        /*
         * For 32-bit usage: A [mG]= (SF/65536) * C
         * For 16-bit usage: A [mG]= SF * C
         */

        double A = (double)rawdata;
        double scale = fmt->scale; //< in_accel_scale

        if ( fmt->length == 32 ) {
                scale /= 65536.0f;
        }
        A *= scale;
        sprintf( strtmp, "%+.8f", A );

        return strtmp;
}

char * get_count( int64_t rawdata, const struct iio_data_format * fmt )
{
        sprintf( strtmp, "%05lld", rawdata );

        return strtmp;
}

char * get_time ( int64_t rawdata, const struct iio_data_format * fmt )
{
        if ( log_start_time == 0 ) {
                log_start_time = rawdata;
        }
        log_stop_time = rawdata;

        sprintf( strtmp, "%.6f", (double)rawdata / 1000000000.0f);

        return strtmp;
}

uint64_t GetTimeStamp()
{
    struct timeval tv;

    gettimeofday(&tv,NULL);

    return tv.tv_sec*(uint64_t)1000000000+tv.tv_usec*1000; /* in nS */
}

unsigned int fir_kaiser_tap( unsigned int fs )
{
        unsigned int val = 0;

        fs >>= 2;
        if ( fs > 4 ) {
                val = 0;
        } else if ( fs == 4 ) {
                val = 128;
        } else if ( fs == 3 ) {
                val = 64;
        } else if ( fs == 2 ) {
                val = 32;
        } else {
                val = 0;
        }

        return val;
}

unsigned int fir_kaiser_fc( unsigned int fs )
{
        unsigned int val = 50;

        fs &= 3;
        for (unsigned int i=0; i < fs; i++) {
                val *= 2;
        }

        return val;
}

/* log file */
void init_log( void )
{
        char s[1024];
        time_t tm;
        struct tm * ti;

        time ( &tm );
        ti = localtime ( &tm );

        if ( ! timestamp_ch.enable ) {
            log_start_time = GetTimeStamp();
        }

        if ( log_file_name == NULL ) {
            sprintf( s, "%s_%d-%02d-%02d_%02d:%02d:%02d.csv", product_id, ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday, ti->tm_hour, ti->tm_min, ti->tm_sec);

        } else {
                strcpy( s, log_file_name );
        }

        if ((fptr_log_file = fopen(s, "wb")) == NULL)
        {
                printf("Error! opening log file");
                exit(1);
        }

        sprintf( s, "Creation Date:,%d-%02d-%02d %02d:%02d:%02d,", ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday, ti->tm_hour, ti->tm_min, ti->tm_sec);
        fwrite( s, 1, strlen(s), fptr_log_file);
        sprintf( s, "PROD_ID=%s,VERSION=%s,SERIAL_NUM=%s,,,,,\n", product_id, firmware_version, serial_number );
        fwrite( s, 1, strlen(s), fptr_log_file);
        sprintf( s, "Scaled Data," );                                           fwrite( s, 1, strlen(s), fptr_log_file);

        sprintf( s, "SF_GYRO=%+.8f/2^16 %s/lsb,",  in_anglvel_scale, g_unit );  fwrite( s, 1, strlen(s), fptr_log_file);
        sprintf( s, "SF_ACCL=%+.8f/2^16 %s/lsb,",  in_accel_scale,   a_unit );  fwrite( s, 1, strlen(s), fptr_log_file);
        sprintf( s, "SF_TEMP=%+.8f/2^16 %s/lsb,,,,,,\n", in_tempX_scale,   t_unit );  fwrite( s, 1, strlen(s), fptr_log_file);

        sprintf( s, "Sample No." );
        fwrite( s, 1, strlen(s), fptr_log_file);
        for ( int i=0; i < CH_CNT; i++ ) {
                if ( epix[i]->enable ) {
                        sprintf( s, ",%s[%s]", epix[i]->alt_name, epix[i]->unit );
                        fwrite( s, 1, strlen(s), fptr_log_file);
                }
        }
        for ( int i=0; i < CH_CNT; i++ ) {
                if ( ! epix[i]->enable ) {
                        fwrite( ",", 1, strlen(","), fptr_log_file);
                }
        }
}

void rec_log( unsigned int record_string_n, char * pdata )
{
        if ( fptr_log_file ) {
                static unsigned int cnt = (unsigned int)-1;
                char s[32];
                if ( cnt != record_string_n ) {
                        if ( record_string_n > 0 ) {
                                for ( int i=0; i < CH_CNT; i++ ) {
                                        if ( ! epix[i]->enable ) {
                                                fwrite( ",", 1, strlen(","), fptr_log_file);
                                        }
                                }
                        }
                        fwrite( "\n", 1, strlen("\n"), fptr_log_file);
                        sprintf( s, "%09u", record_string_n );
                        fwrite( s, 1, strlen(s), fptr_log_file);
                        cnt = record_string_n;
                }
                sprintf( s, ",%s", pdata );
                fwrite( s, 1, strlen(s), fptr_log_file);
        }
}

unsigned int mv_avg_tap( unsigned int fs )
{
        unsigned int val = 0;

        if ( fs != 0 ) {
                val = 2;
                fs--;
                for (unsigned int i=0; i < fs; i++) {
                        val *= 2;
                }
        }

        return val;
}

void finish_log( void )
{
        if ( fptr_log_file ) {
                char s[1024];
                char stmp[32];
                char time_location[32] = "imu time";
                time_t start_time, stop_time;
                struct tm  ts;
                ssize_t err;
                long long filter;

                for ( int i=0; i < CH_CNT; i++ ) {
                        if ( ! epix[i]->enable ) {
                                fwrite( ",", 1, strlen(","), fptr_log_file);
                        }
                }

                fwrite( "\n", 1, strlen("\n"), fptr_log_file);

                if ( timestamp_ch.enable == false ) {
                        log_stop_time = GetTimeStamp();
                        strcpy( time_location, "console time" );
                }

                start_time = (time_t)(log_start_time / 1000000000);
                stop_time  = (time_t)(log_stop_time  / 1000000000);

                /* start time */
                ts = *localtime((time_t*) &start_time);
                sprintf(s, "Log Start[%s],", time_location );
                fwrite( s, 1, strlen(s), fptr_log_file);
                strftime(s, sizeof(s), "%Y-%m-%d %H:%M:%S", &ts); //< "yyyy-mm-dd hh:mm:ss"
                fwrite( s, 1, strlen(s), fptr_log_file);
                sprintf(s, ".%09lld,,,,,,,,\n", log_start_time % 1000000000);
                fwrite( s, 1, strlen(s), fptr_log_file);

                /* stop time */
                ts = *localtime((time_t*) &stop_time);
                sprintf(s, "Log End  [%s],", time_location );
                fwrite( s, 1, strlen(s), fptr_log_file);
                strftime(s, sizeof(s), "%Y-%m-%d %H:%M:%S", &ts); //< "yyyy-mm-dd hh:mm:ss"
                fwrite( s, 1, strlen(s), fptr_log_file);
                sprintf(s, ".%09lld,,,,,,,,\n", log_stop_time % 1000000000);
                fwrite( s, 1, strlen(s), fptr_log_file);

                /* duration */
                sprintf(s, "Log Duration,%.6f,,,,,,,,\n", (double)(log_stop_time - log_start_time) / 1000000000.0f);
                fwrite( s, 1, strlen(s), fptr_log_file);

                /* expected rate */
                err = iio_device_attr_read(dev, "sampling_frequency", stmp, sizeof(stmp));
                if ( err < 0 ) {
                        printf("Failed to read sampling_frequency\n");
                }
                sprintf(s, "Expected Rate,%s,Filter Cfg,", stmp);
                fwrite( s, 1, strlen(s), fptr_log_file);

                /* filter */
                err = iio_channel_attr_read_longlong(anglvel_ch_x.chn, "filter_low_pass_3db_frequency", &filter);
                if ( err < 0 ) {
                        printf("Could not to read filter_low_pass_3db_frequency\n");
                } else if ( filter < 8 ) {
                        sprintf(s, "MV_AVG%u,,,,,,\n", mv_avg_tap(filter&0xff));
                } else {
                        sprintf(s, "FIR_Kaiser%u_fc%u,,,,,,\n", fir_kaiser_tap(filter), fir_kaiser_fc(filter));
                }
                fwrite( s, 1, strlen(s), fptr_log_file);

                fclose(fptr_log_file);
        }
}

/* cleanup and exit */
void shutdown()
{
        unsigned int i;

        finish_log();

        printf("* Disabling IIO streaming channels (channel_count=%u) for buffered capture\n", channel_count);
        for (i = 0; i < channel_count; ++i) {
                if ( epix[i]->chn )
                        iio_channel_disable(epix[i]->chn);
        }

        printf("* Destroying buffers\n");
        if (rxbuf) { iio_buffer_destroy(rxbuf); }

        printf("* Disassociate trigger\n");
        if (dev) {
                int ret = iio_device_set_trigger(dev, NULL);
                if (ret < 0) {
                        char buf[256];
                        iio_strerror(-ret, buf, sizeof(buf));
                        fprintf(stderr, "%s (%d) while Disassociate trigger\n", buf, ret);
                }
        }

        printf("* Destroying context\n");
        if (ctx) { iio_context_destroy(ctx); }
        exit(0);
}

void handle_sig(int sig)
{
    printf("Waiting for process to finish... got signal : %d\n", sig);
    stop = true;
}

void usage(__notused int argc, char *argv[])
{
        printf("Usage: %s [OPTION]\n", argv[0]);
        printf("  -d\tdevice name (default %s)\n", device_name);
        printf("  -t\ttrigger name (default %s)\n", trigger_name);
        printf("  -b\tbuffer length (default 1)\n");
        printf("  -e\tenable channels (default all: tcsga)\n");
        printf("  \t\t t - Temperature\n");
        printf("  \t\t c - Counter\n");
        printf("  \t\t s - Timestamp\n");
        printf("  \t\t g - Gyro (x,y,z)\n");
        printf("  \t\t a - Accel (x,y,z)\n");
        printf("  -c\tread count (default no limit)\n");
        printf("  -s\tsample rate (default as is)\n");
        printf("  -f\tfilter select (default as is)\n");
        printf("  -n\tremote host (default as is)\n");
        printf("  -l\twrite log file. This must be the last arg if no file name provided.\n");
}

void parse_options(int argc, char *argv[])
{
        int c;

        while ((c = getopt(argc, argv, ":d:t:b:c:h:s:e:f:l:n:")) != -1) {
                switch (c)
                {
                case 'd':
                        device_name = optarg;
                        break;
                case 't':
                        trigger_name = optarg;
                        break;
                case 'b':
                        buffer_length = atoi(optarg);
                        break;
                case 'n':
                        remote_host = optarg;
                        printf( "remote_host:%s\n", remote_host);
                        break;
                case 'c':
                        if (atoi(optarg) > 0) {
                                count = atoi(optarg);
                        } else {
                                usage(argc, argv);
                                exit(1);
                        }
                        break;
                case 's':
                        sample_rate = optarg;
                        break;
                case 'f':
                        filter_sel = optarg;
                        break;
                case 'e':
                        if (!strstr(optarg, "a") && !strstr(optarg, "g")) {
                                printf("Gyro and Accel cannot be disabled both at same time.\n");
                                exit(1);
                        }
                        temp_ch.enable      = false;
                        anglvel_ch_x.enable = false;
                        anglvel_ch_y.enable = false;
                        anglvel_ch_z.enable = false;
                        accel_ch_x.enable   = false;
                        accel_ch_y.enable   = false;
                        accel_ch_z.enable   = false;
                        count_ch.enable     = false;
                        timestamp_ch.enable = false;

                        if (strstr(optarg, "t")) temp_ch.enable      = true;
                        if (strstr(optarg, "c")) count_ch.enable     = true;
                        if (strstr(optarg, "s")) timestamp_ch.enable = true;
                        if (strstr(optarg, "g")) {
                                anglvel_ch_x.enable   = true;
                                anglvel_ch_y.enable   = true;
                                anglvel_ch_z.enable   = true;
                        }
                        if (strstr(optarg, "a")) {
                                accel_ch_x.enable     = true;
                                accel_ch_y.enable     = true;
                                accel_ch_z.enable     = true;
                        }
                        break;
                case 'l':
                        logging_data = true;
                        if ( optarg[0] != '-' )
                                log_file_name = optarg;
                        break;
                case ':':
                        switch (optopt)
                        {
                                case 'l':
                                        printf("- Logging data is ON to default file name\n");
                                        logging_data = true;
                                        break;
                                default:
                                        fprintf(stderr, "option -%c is missing a required argument\n", optopt);
                                        usage(argc, argv);
                                        exit(1);
                        }
                        break;
                default:
                        usage(argc, argv);
                        exit(1);
                }
        }
}


/* simple configuration and streaming */
int main (int argc, char **argv)
{
        ssize_t err;

        /* Hardware trigger */
        const struct iio_device *trigger;

        parse_options(argc, argv);

        /* Listen to ctrl+c and assert */
        signal(SIGINT, handle_sig);

        unsigned int i, j, major, minor;
        char git_tag[8];
        iio_library_get_version(&major, &minor, git_tag);
        printf("Library version: %u.%u (git tag: %s)\n", major, minor, git_tag);

        /* check for struct iio_data_format.repeat support
         * 0.8 has repeat support, so anything greater than that */
        has_repeat = ((major * 10000) + minor) >= 8 ? true : false;

        if ( remote_host != NULL ) {
                printf("* Acquiring IIO context from the network: %s\n", remote_host);
                ctx = iio_create_network_context(remote_host);
        } else {
                printf("* Acquiring IIO default context\n");
                /* this also tries network through IIOD_REMOTE environmental variable */
                ctx = iio_create_default_context();
        }

        if ( ctx == NULL ) {
                printf("No context\n");
                exit(0);
        }

        if ( iio_context_get_devices_count(ctx) == 0 ) {
                printf("No devices\n");
                shutdown();
        }

        printf("* Acquiring device %s\n", device_name);
        dev = iio_context_find_device(ctx, device_name);
        if (!dev) {
                printf("No device found\n");
                shutdown();
        }

        /* product_id */
        err = iio_device_attr_read(dev, "product_id", product_id, sizeof(product_id));
        if ( err < 0 ) {
                printf("Failed to read product_id\n");
                shutdown();
        }
        printf("* product_id: %s\n", product_id );

        /* serial_number */
        err = iio_device_attr_read(dev, "serial_number", serial_number, sizeof(serial_number));
        if ( err < 0 ) {
                printf("Failed to read serial_number\n");
                shutdown();
        }
        printf("* serial_number: %s\n", serial_number );

        /* firmware_version */
        err = iio_device_attr_read(dev, "firmware_version", firmware_version, sizeof(firmware_version));
        if ( err < 0 ) {
                printf("Failed to read firmware_version\n");
                shutdown();
        }
        printf("* firmware_version: %s\n", firmware_version );

        printf("* Initializing IIO streaming channels:\n");
        for (i = 0; i < iio_device_get_channels_count(dev); ++i) {
                struct iio_channel *chn = iio_device_get_channel(dev, i);
                if (iio_channel_is_scan_element(chn)) {
                        printf("%s\n", iio_channel_get_id(chn));
                        channel_count++;
                }
        }
        if (channel_count != 9) {
                printf("Scan elements count mismatch: %u (expected 9)\n", channel_count);
                shutdown();
        }

        for (i = 0; i < channel_count; ++i) {
                struct iio_channel *chn = iio_device_get_channel(dev, i);
                if (iio_channel_is_scan_element(chn) ) {
                        char ch_name[128];
                        sprintf( ch_name, iio_channel_get_id(chn) );

                        if (      strstr(ch_name,  anglvel_ch_x.attr_name) ) epix[i] = &anglvel_ch_x;
                        else if ( strstr(ch_name,  anglvel_ch_y.attr_name) ) epix[i] = &anglvel_ch_y;
                        else if ( strstr(ch_name,  anglvel_ch_z.attr_name) ) epix[i] = &anglvel_ch_z;
                        else if ( strstr(ch_name,    accel_ch_x.attr_name) ) epix[i] = &accel_ch_x;
                        else if ( strstr(ch_name,    accel_ch_y.attr_name) ) epix[i] = &accel_ch_y;
                        else if ( strstr(ch_name,    accel_ch_z.attr_name) ) epix[i] = &accel_ch_z;
                        else if ( strstr(ch_name,      count_ch.attr_name) ) epix[i] = &count_ch;
                        else if ( strstr(ch_name,  timestamp_ch.attr_name) ) epix[i] = &timestamp_ch;
                        else if ( strstr(ch_name,       temp_ch.attr_name) ) epix[i] = &temp_ch;
                        else shutdown();

                        epix[i]->chn = chn;
                }
        }

        /* in_temp0_offset */
        if ( temp_ch.chn != NULL ) {
                if( iio_channel_attr_read_double(temp_ch.chn, "offset", &in_tempX_offset) ) {
                        printf("Could not read in_temp0_offset\n");
                        shutdown();
                }
                if( iio_channel_attr_read_double(temp_ch.chn, "scale", &in_tempX_scale) ) {
                        printf("Could not read in_temp0_scale\n");
                        shutdown();
                }
                in_tempX_scale /= 1000.0f;
        } else {
                printf("Could not read in_temp0 channel\n");
                shutdown();
        }

        /* in_accel_scale */
        if ( accel_ch_x.chn != NULL ) {
                if( iio_channel_attr_read_double(accel_ch_x.chn, "scale", &in_accel_scale) ) {
                        printf("Could not read in_accel_scale\n");
                        shutdown();
                }
        } else {
                printf("Could not read in_accel_scale\n");
                shutdown();
        }

        /* in_anglvel_scale */
        if ( anglvel_ch_x.chn != NULL ) {
                if( iio_channel_attr_read_double(anglvel_ch_x.chn, "scale", &in_anglvel_scale) ) {
                        printf("Could not read in_anglvel_scale\n");
                        shutdown();
                }
        } else {
                printf("Could not read in_anglvel_scale\n");
                shutdown();
        }

        /* sampling_frequency */
        if ( sample_rate != NULL ) {
                err = iio_device_attr_write(dev, "sampling_frequency", sample_rate);
                if ( err < 0 ) {
                        printf("Could not set sampling_frequency\n");
                }
        }
        err = iio_device_attr_read(dev, "sampling_frequency", strtmp, STRTMP_SIZE);
        if ( err < 0 ) {
                printf("Could not read sampling_frequency\n");
                shutdown();
        } else {
                printf("* sampling_frequency: %s Sps\n", strtmp );
        }

        /* filter_low_pass_3db_frequency */
        if ( anglvel_ch_x.chn != NULL ) {
                if ( filter_sel != NULL ) {
                        err = iio_channel_attr_write(anglvel_ch_x.chn, "filter_low_pass_3db_frequency", filter_sel );
                        if ( err < 0 ) {
                                printf("Could not set sampling_frequency\n");
                                shutdown();
                        }
                }

                err = iio_channel_attr_read(anglvel_ch_x.chn, "filter_low_pass_3db_frequency", strtmp, STRTMP_SIZE);
                if ( err < 0 ) {
                        printf("Could not read filter_low_pass_3db_frequency\n");
                        shutdown();
                } else {
                        printf("* filter_low_pass_3db_frequency: %s\n", strtmp );
                }
        } else {
                printf("Could not find anglvel_x\n");
        }

        printf("* Acquiring trigger %s\n", trigger_name);
        trigger = iio_context_find_device(ctx, trigger_name);
        if (!trigger || !iio_device_is_trigger(trigger)) {
                printf("No trigger found (try setting up the iio-trig-hrtimer module)\n");
                shutdown();
        }

        printf("* Enabling IIO streaming channels (channel_count=%u) for buffered capture\n", channel_count);
        for (i = 0; i < channel_count; ++i) {
                if ( epix[i]->enable == true ) {
                        iio_channel_enable(epix[i]->chn);
                } else {
                        iio_channel_disable(epix[i]->chn);
                }
        }

        printf("* Enabling IIO buffer trigger\n");
        if (iio_device_set_trigger(dev, trigger)) {
                printf("Could not set trigger\n");
                shutdown();
        }

        printf("* Creating non-cyclic IIO buffers with %d samples\n", buffer_length);
        rxbuf = iio_device_create_buffer(dev, buffer_length, false);
        if (!rxbuf) {
                printf("Could not create buffer\n");
                shutdown();
        }

        printf("* Starting IO streaming (press CTRL+C to cancel)\n");
        int64_t last_ts = 0;
        unsigned int record_line = 0;

        if ( logging_data ) {
                init_log();
        }

        while (!stop)
        {
                /* Refill RX buffer */
                ssize_t nbytes_rx = iio_buffer_refill(rxbuf);

                if (nbytes_rx < 0) {
                        printf("Error refilling buf: %d\n", (int)nbytes_rx);
                        shutdown();
                }

                /* Print timestamp delta in ms */
                if ( timestamp_ch.enable == true ) {
                        int64_t now_ts;
                        size_t bytes = iio_channel_read(timestamp_ch.chn, rxbuf, &now_ts, sizeof(int64_t) * buffer_length);
                        printf("[%04" PRId64 "] ", last_ts > 0 ? (now_ts - last_ts)/1000/1000 : 0);
                        last_ts = now_ts;
                }

                for (i = 0; i < channel_count; ++i) {
                        int64_t ch_data = 0;
                        uint8_t *buf;
                        size_t sample, bytes;
                        if ( epix[i]->enable == false ) continue;
                        const struct iio_data_format *fmt = iio_channel_get_data_format(epix[i]->chn);
                        unsigned int repeat = has_repeat ? fmt->repeat : 1;
                        size_t sample_size = fmt->length / 8 * repeat;

                        buf = malloc(sample_size * buffer_length);
                        if (!buf) {
                                printf("trying to allocate memory for buffer\n");
                                shutdown();
                        }

                        bytes = iio_channel_read(epix[i]->chn, rxbuf, buf, sample_size * buffer_length);

                        for (sample = 0; sample < bytes / sample_size; ++sample) {
                                for (j = 0; j < repeat; ++j) {
                                        if (fmt->length / 8 == sizeof(int16_t)) {
                                                ch_data = ((int16_t *)buf)[sample+j];
                                        } else if (fmt->length / 8 == sizeof(int32_t)) {
                                                ch_data = ((int32_t *)buf)[sample+j];
                                        } else if (fmt->length / 8 == sizeof(int64_t)) {
                                                ch_data = ((int64_t *)buf)[sample+j];
                                        }

                                }
                        }
                        char * pstr = epix[i]->fptr( ch_data, fmt );
                        printf(" %s:%s", epix[i]->alt_name, pstr);
                        rec_log( record_line, pstr );
                        free(buf);
                }
                printf("\n");

                if (count != -1 && --count == 0)
                        break;

                record_line++;
        }

        shutdown();

        return 0;
}


