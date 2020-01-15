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
* hbipc_ap_init - Init connection between client APP & HBIPC_AP module
* @domain_name: communication domain name
* @flags: transfer feature
* @timeout: user set timeout in mircoseconds
* @domain_cfg: domain configuration information
* return: on success return a valid domain id; if fail return negative value
*/
int hbipc_ap_init(IN const char *domain_name, IN int flags, IN int timeout,
IN struct domain_configuration *domain_cfg);

/*
* hbipc_ap_deinit - Deinit connection between client APP & HBIPC_AP module
* @domain_id: communication id which get from HBIPC_AP_Init function
* return: on success return 0; if fail return negative value
*/
int hbipc_ap_deinit(IN int domain_id);

/*
* hbipc_ap_startserver - Start a provider app specified by server_id & parameter
* @ domain_id: communication id which get from HBIPC_AP_Init function
* @ server_id: specify provider app type
* @ parameter: provider app boot parameter, if set NULL, start server with default parameters
* @ flag: wait feature
* return: on success return a valid server instance id; if fail return negative value
*/
int hbipc_ap_startserver(IN int domain_id, IN uuid server_id, IN char *parameter, IN int flags);

/*
* hbipc_ap_stopserver - Stop a provider app specified by instance id
* @domain_id: communication id which get from HBIPC_AP_Init function
* @provider_id: server instance id which get from HBIPC_AP_StartServer function
* return: on success return 0; if fail return negative value
*/
int hbipc_ap_stopserver(IN int domain_id, IN int provider_id);

/*
* hbipc_ap_connect - Establish connection between AP client app & CP provider app
* @domain_id: communication id which get from HBIPC_AP_Init function
* @provider_id: server instance id which get from HBIPC_AP_StartServer function
* @session: contain session information if establish connection successfully
* return: on success return 0; if fail return negative value
*/
int hbipc_ap_connect(IN int domain_id, IN int provider_id, OUT struct session *session);

/*
* hbipc_ap_disconnect - Disconnect the link between AP client app & CP provider app
* session: session information which get from HBIPC_AP_Connect function
* return: on success return 0; if fail return negative value
*/
int hbipc_ap_disconnect(IN struct session *session);

/*
* hbipc_ap_sendframe - Send frame to provider app in X2 port
* @session: session information which get from HBIPC_AP_Connect function
* @buf: contain frame data want to send
* @len: data length want to send
* return: on success return 0; if fail return negative value
*/
int hbipc_ap_sendframe(IN struct session *session, IN char *buf, IN int len);

/*
* hbipc_ap_recvframe - receive frame from provider app in X2 port
* @session: session information which get from HBIPC_AP_Connect function
* @buf: contain received data
* @len: received data length
* return: on success return the number of bytes received; if faill return negative value
*/
int hbipc_ap_recvframe(IN struct session *session, OUT char *buf, IN int len);

#ifdef __cplusplus
}
#endif

#endif