/*!
 * \cond FILEINFO
 ******************************************************************************
 * \file at24mac402.h
 ******************************************************************************
 * Copyright (C) embeddeers GmbH, 2020
 ******************************************************************************
 *
 * \brief  AT24MAC402 read serial number API
 *
 * \par Purpose
 *      
 *
 * $LastChangedRevision: 1905 $  embeddeers revision number
 *
 ******************************************************************************
 *
 * \endcond
 */

#ifndef EMB_AT24MAC402_H_
#define EMB_AT24MAC402_H_

char* at24mac402_readMac();
char* at24mac402_readUid();
char* at24mac402_getUidStr();

#endif /* EMB_AT24MAC402_H_ */
