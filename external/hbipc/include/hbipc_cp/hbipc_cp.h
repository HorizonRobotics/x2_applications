/*
 *			 COPYRIGHT NOTICE
 *		 Copyright 2019 Horizon Robotics, Inc.
 *			 All rights reserved.
 */

#ifndef _HBIPC_AP_H
#define _HBIPC_AP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>

#define VERSION "2.3.0"

#define IN
#define OUT

#define DOMAIN_MAX_NUM (6)
struct domain_info {
	char *domain_name;
	int domain_id;
	char *device_name;
};

struct domain_configuration {
	int domain_number;
	struct domain_info domain_array[DOMAIN_MAX_NUM];
};

#define UUID_LEN (16)
typedef unsigned char uuid[UUID_LEN];

#define UUID_DEFINE(name, u0, u1, u2, u3, u4, u5, u6, u7, u8, u9, u10, u11, u12, u13, u14, u15)\
	uuid name = {u0, u1, u2, u3, u4, u5, u6, u7, u8, u9, u10, u11, u12, u13, u14, u15}

struct session {
	int domain_id;
	int provider_id;
	int client_id;
};

/* transfer feature */
#define TF_NONBLOCK (1 << 0)
#define TF_BLOCK    ~(1 << 0)

/*
* hbipc_cp_init - Init connection between client APP & HBIPC_AP module
* @domain_name: communication domain name
* @server_id: provider app server id want to register
* @flags: transfer feature
* @timeout: user set timeout in microseconds
* @domain_cfg: domain configuration information
* return: on success return a valid domain id; if fail return negative value
*/
int hbipc_cp_init(IN const char *domain_name, IN uuid server_id, IN int flags,
IN int timeout, IN struct domain_configuration *domain_cfg);

/*
* hbipc_cp_deinit - Deinit connection between client APP & HBIPC_AP module
* @domain_id: communication id which get from HBIPC_AP_Init function
* @server_id: provider app server id want to register
* return: on success return 0; if fail return negative value
*/
int hbipc_cp_deinit(IN int domain_id, IN uuid server_id);

/*
* hbipc_cp_accept - Accept connection request from AP client app
* @domain_id: communication id which get from HBIPC_AP_Init function
* @server_id: provider app server_id want to accept connection
* @session: contain session information if establish connection successfully
* @flags: wait feature
* return: on success return 0; if fail return negative value
*/
int hbipc_cp_accept(IN int domain_id, IN uuid server_id, OUT struct session *session, IN int flags);

/*
* hbipc_cp_sendframe - Send frame to client app in AP port
* @session: session information which get from HBIPC_AP_Connect function
* @buf: contain frame data want to send
* @len: data length want to send
* return: on success return 0; if fail return negative value
*/
int hbipc_cp_sendframe(IN struct session *session, IN char *buf, IN int len);

/*
* hbipc_cp_recvframe - receive frame from clien app in AP port
* @session: session information which get from HBIPC_X2_Accept function
* @buf: contain received data
* @len: received data length
* return: on success return the number of bytes received; if faill return negative value
*/
int hbipc_cp_recvframe(IN struct session *session, OUT char *buf, IN int len);

#ifdef __cplusplus
}
#endif

#endif