/*
 * Copyright (c) 2014, 2015 Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file uart_test.c
 * @brief Ara Toshiba bridge ASIC uart test program
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>

#include <apps/greybus-utils/utils.h>

#include <nuttx/device.h>
#include <nuttx/device_uart.h>


/**
 * @brief The modem status change callback function.
 *
 * For verifying the driver can correctly call this callback.
 *
 * @param ms The present UART modem status.
 * @return None.
 */
static void ms_callback(uint8_t ms)
{
  
   gb_info("%s():   +++ %x    \n", __func__,ms);
    
}

/**
 * @brief The line status change callback function.
 *
 * For verifying the driver can correctly call this callback.
 *
 * @param ls The present UART line status.
 * @return None.
 */
static void ls_callback(uint8_t ls)
{
   
   gb_info("%s():   +++ %x    \n", __func__,ls);
     
}



/**
 * @brief The transmit callback function.
 *
 * For verifying the driver can correctly call this callback.
 *
 * @param ls The present UART line status.
 * @return None.
 */
static void tx_callback(uint8_t *buffer, int length, int error)
{
  //  fprintf(stderr, "tx_callback:\n");
   // fprintf(stderr, " data = %s, length = %d, error = %d\n",
   //         buffer, length, error);
  //gb_info("%s():   +++  length %d    \n", __func__,length);
    gb_info("%s():   +++ data = %s, length = %d, error = %d    \n", __func__,buffer, length, error);
}

/**
 * @brief The set modem control test function
 *
 * Calls the set modem control function, the driver should trun on the debug
 * message to show the register value for verifying.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_set_modem_ctrl(struct device *dev)
{
    int ret;
    uint8_t modem_ctrl;

    modem_ctrl = (MCR_RTS | MCR_LPBK);
   
    gb_info("%s(): set modem control %x    \n", __func__,modem_ctrl);

    ret = device_uart_set_modem_ctrl(dev, &modem_ctrl);
    if (ret) {
        gb_info("%s(): device_uart_set_modem_ctrl failed: %d   \n", __func__,ret);
        return ret;
    }
    return 0;
}

/**
 * @brief The get modem control test function
 *
 * Calls the get modem control function to get modem control value.
 * Driver may turn on the debug message to show the register values, user could
 * compare the register value and the returned value.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_get_modem_ctrl(struct device *dev)
{
    int ret;
    uint8_t modem_ctrl = 0;

    ret = device_uart_get_modem_ctrl(dev, &modem_ctrl);
    if (ret) {
        gb_info("%s(): device_uart_get_modem_ctrl failed: %d   \n", __func__,ret);
        return ret;
    }

    gb_info("%s(): get modem control %x    \n", __func__,modem_ctrl);
    return 0;
}

/**
 * @brief The get modem status test function
 *
 * Calls the get modem status function to get modem status value.
 * Driver may turn on the debug message to show the register values, user could
 * compare the register value and the returned value.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_get_modem_status(struct device *dev)
{
    int ret;
    uint8_t modem_status = 0;

    ret = device_uart_get_modem_status(dev, &modem_status);
    if (ret) {
        gb_info("%s(): device_uart_get_modem_status failed: %d   \n", __func__,ret);
        return ret;
    }
    gb_info("%s(): get modem status %x    \n", __func__,modem_status);
    return 0;
}

/**
 * @brief The get line status test function
 *
 * Calls the get line status function to get line status value.
 * Driver may turn on the debug message to show the register values, user could
 * compare the register value and the returned value.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_get_line_status(struct device *dev)
{
    int ret;
    uint8_t line_status = 0;

    ret = device_uart_get_line_status(dev, &line_status);
    if (ret) {
        gb_info("%s(): device_uart_get_line_status failed: %d   \n", __func__,ret);
        return ret;
    }
    gb_info("%s(): get line status %x    \n", __func__,line_status);
    return 0;
}

/**
 * @brief The set break test function
 *
 * Calls the set break function to set break state.
 * Driver may turn on the debug message to show the register values, user could
 * compare the register value and the setting value.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_set_break_on(struct device *dev)
{
    int ret;

    ret = device_uart_set_break(dev, 1);
    if (ret) {
        gb_info("%s():  failed: %d \n", __func__,ret);
        return ret;
    }


    return 0;
}


static int do_test_set_break_off(struct device *dev)
{
    int ret;

    ret = device_uart_set_break(dev, 0);
    if (ret) {
        gb_info("%s():  failed: %d \n", __func__,ret);
        return ret;
    }
    return 0;
}

/**
 * @brief The attach modem status change callback test function
 *
 * First set the callback NULL, user should observe the driver turn off the
 * modem status change interrupt. And it should be enabled if the callback in
 * not NULL and be called if modem status changed.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_attach_ms_callback(struct device *dev)
{
    int ret;

    ret = device_uart_attach_ms_callback(dev, NULL);
    if (ret) {
        //fprintf(stderr, "device_uart_attach_ms_callback NULL failed: %d \n",
        //       ret);
        gb_info("%s():  NULL failed: %d \n", __func__,ret);
        return ret;
    }

    ret = device_uart_attach_ms_callback(dev, ms_callback);
    if (ret) {
        //fprintf(stderr, "device_uart_attach_ms_callback with callback: %d\n",
        //       ret);
        gb_info("%s():  with callback:: %d \n", __func__,ret);       
        return ret;
    }
    return 0;
}

/**
 * @brief The attach line status change callback test function
 *
 * First set the callback NULL, user should observe the driver turn off the
 * line status change interrupt. And it should be enabled if the callback in
 * not NULL and be called if line status changed.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_attach_ls_callback(struct device *dev)
{
    int ret;

    ret = device_uart_attach_ls_callback(dev, NULL);
    if (ret) {
        //fprintf(stderr, "device_uart_attach_ms_callback NULL failed: %d \n",
         //      ret);
         gb_info("%s():  NULL failed: %d \n", __func__,ret);      
        return ret;
    }

    ret = device_uart_attach_ls_callback(dev, ls_callback);
    if (ret) {
        //fprintf(stderr, "device_uart_attach_ms_callback with callback: %d\n",
        //       ret);
        gb_info("%s():  with callback:: %d \n", __func__,ret);  
        return ret;
    }
    return 0;
}

/**
 * @brief The data transmit test.
 *
 * Send data out in block mode. Check the serial port terminal for the data.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_data_transmit(struct device *dev)
{
    int ret, xmit = 0;
    uint8_t tx_buf[] = "transmit test";

	//fprintf(stderr, "device_uart_start_transmitter ++: \n" );
	gb_info("%s(): ++: \n", __func__);
    ret = device_uart_start_transmitter(dev, tx_buf, sizeof(tx_buf), NULL,
                                        &xmit, NULL);
    gb_info("%s(): --: \n", __func__);
    if (ret) {
        //fprintf(stderr, "device_uart_start_transmitter failed: %d\n", ret);
         gb_info("%s(): device_uart_start_transmitter failed: %d \n", __func__,ret);
        return ret;
    }

    //fprintf(stderr, "device_uart_start_transmitter: sent = %d\n", xmit);
    //gb_info("%s(): device_uart_start_transmitter sent: %d \n", __func__,xmit);

    return 0;
}

static void rx_callback(uint8_t *buffer, int length, int error);

/**
 * @brief The data receive test.
 *
 * Receive data from uart port and verify the data.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_data_receive(struct device *dev)
{
    int ret, recv = 0;
    uint8_t rx_buf[16];
   // memset(&rx_buf,0x00,sizeof(rx_buf));

    ret = device_uart_start_receiver(dev, rx_buf, 16, NULL, NULL, rx_callback);
    if (ret) {
        //fprintf(stderr, "device_uart_start_receiver failed: %d\n", ret);
        gb_info("%s(): device_uart_start_receiver failed: %d \n", __func__,ret);
        return ret;
    }

    rx_buf[recv] = 0x00; /* string end */
    //fprintf(stderr, "device_uart_start_receiver: data = %s, recv = %d",
    //        rx_buf, recv);
	//gb_info("%s():device_uart_start_receiver: data = %s, recv = %d \n", __func__,rx_buf, recv);
    return 0;
}

/**
 * @brief The data loopback tranfer test.
 *
 * First set the callback NULL, user should observe the driver turn off the
 * line status change interrupt. And it should be enabled if the callback in
 * not NULL and be called if line status changed.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
static int do_test_data_loopback(struct device *dev)
{
    int ret;
    uint8_t rx_buf[16];
    uint8_t tx_buf[] = "xmit test";

    /*
     * ret = device_uart_set_modem_ctrl(dev, MCR_LPBK);
     * if (!ret) {
     *     fprint(stderr, "device_uart_set_modem_ctrl \n");
     *   return ret;
     * }
     */

    ret = device_uart_start_receiver(dev, rx_buf, 16, NULL, NULL,
                                     &rx_callback);
    if (!ret) {
        //fprintf(stderr, "device_uart_start_receiver failed: %d\n", ret);
        gb_info("%s(): device_uart_start_receiver failed: %d \n", __func__,ret);
        return ret;
    }

    ret = device_uart_start_transmitter(dev, tx_buf, sizeof(tx_buf), NULL,
                                        NULL, &tx_callback);
    if (!ret) {
        //fprintf(stderr, "device_uart_start_transmitter failed: %d\n", ret);
        gb_info("%s(): device_uart_start_receiver failed: %d \n", __func__,ret);
        return ret;
    }

    return 0;
}




/**
 * @brief Test all
 *
 * To solve un-used functions warnings.
 *
 * @param dev The device driver handler.
 * @return None.
 */
static void do_all_test(struct device *dev)
{
   // do_test_set_modem_ctrl(dev);
   // do_test_get_modem_ctrl(dev);
    //do_test_get_modem_status(dev);
    //do_test_get_line_status(dev);
    //do_test_set_break(dev);
    do_test_attach_ms_callback(dev);
    do_test_attach_ls_callback(dev);
    do_test_data_transmit(dev);
    do_test_data_receive(dev);
    //do_test_data_loopback(dev);
}



enum {
    HELP,
    SEND_DATA,
    RECEIVE_DATA,
    SET_LINE_CODING,
    SET_CONTROL_LINE_STATE,
    GET_CONTROL_LINE_STATE,
    SEND_BREAK_ON,
    SEND_BREAK_OFF,
    SERIAL_STATE,
    MAX_CMD,
};

struct command {
    const char shortc;
    const char *longc;
    const char *help;
};

static const struct command commands[] = {
    [HELP]					= {'h', "help", "Print this message and exit"},
    [SEND_DATA]				= {'s', "send-data", "Send data to UART"},
    [RECEIVE_DATA]			= {'r', "receive-data", "receive data from UART"},
    [SET_LINE_CODING]		= {'l', "set-line-coding","set line coding to UART"},
    [SET_CONTROL_LINE_STATE]= {'c', "set-control-state","set control line to UART"},
    [GET_CONTROL_LINE_STATE]= {'C', "get-control-state","get control line to UART"},
    [SEND_BREAK_ON]			= {'B', "send-break", "send break ON to UART"},
    [SEND_BREAK_OFF]		= {'b', "send-break", "send break ON to UART"},
    [SERIAL_STATE]			= {'e', "serial-state", "Get serial state value"},
};


static void print_usage(void)
{
    int i;
  
    gb_info("%s(): uart: usage: ++ \n", __func__);
    for (i = 0; i < MAX_CMD; i++) {
        const char *space = " ";

        gb_info("%s():uart_test [%c|%s]%s: %s \n", __func__,
               commands[i].shortc, commands[i].longc,
               space,
               commands[i].help);
    }
    gb_info("%s(): uart: usage: --  \n", __func__);       
}

static void usage(int exit_status)
{
    print_usage();
    exit(exit_status);
}

static void get_protocol_version(void)
{

	printf("get_protocol_version\n");
    //printf("uart line count: %u\n", gpio_line_count());
}

struct device *dev;

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int uart_test_main(int argc, char *argv[])
#endif
{
#if 1
   // struct device *dev;
    int ret;
   
  
	gb_info("%s():  hello  +++   \n", __func__);


    dev = device_open(DEVICE_TYPE_UART_HW, 0);
    if (dev == NULL) {
        gb_info("%s(): open failed \n", __func__);
        return -EIO;
    }

    ret = device_uart_set_configuration(dev, BAUD_115200, NO_PARITY, 8,
                                        ONE_STOP_BIT, 0);
    if (ret) {
        gb_info("%s(): device_uart_set_configuration failed %d \n", __func__,ret);
        goto err_device_opened;
    }
    
 
   // do_all_test(dev);
    /*
    ret = do_test_data_loopback(dev);
    if (ret) {
        fprintf(stderr, "uart_test_main failed: %d\n", ret);
    }
	*/

	do_test_data_transmit(dev);

	do_test_data_receive(dev);

err_device_opened:
  //  device_close(dev);

    return 0;

#else

    int i;
    int cmd = INVALID;
    const char *cmd_str;
    int rc = 0;

    /* Parse arguments. */
    if (argc < 2) {
        rc = EXIT_FAILURE;
        goto done;
    }
    cmd_str = argv[1];
    for (i = 0; i < MAX_CMD; i++) {
        if (!strcmp(cmd_str, commands[i].longc)) {
            cmd = i;
            break;
        } else if (strlen(cmd_str) == 1 &&
                   cmd_str[0] == commands[i].shortc) {
            cmd = i;
            break;
        }
    }

    /* Parse command arguments and run command. */
    argc -= 2;
    void (*handler)(uint8_t*, size_t);
    switch (cmd) {
    /* These are special cases. */
    case HELP:
        print_usage();
        goto done;
    case PROTOCOL_VERSION:
        get_protocol_version();
        goto done;
        
        
 default:
        rc = EXIT_FAILURE;
        goto done;
    case SEND_DATA:
  /*      if (argc == 0) {
            fprintf(stderr,
                    "You must specify at least one GPIO and value to set.\n");
            rc = EXIT_FAILURE;
            goto done;
        } else if (argc & 1) {
            fprintf(stderr, "You must specify one value per GPIO\n");
            rc = EXIT_FAILURE;
            goto done;
        } else {
            printf("Setting (gpio, value):");
            for (i = 0; i < argc / 2; i++) {
                gpios[i] = (uint8_t)atoi(argv[2 + 2 * i]);
                values[i] = !!(uint8_t)atoi(argv[3 + 2 * i]);
                printf(" (%u, %u)", gpios[i], values[i]);
            }
            printf("\n");
            do_set_value(gpios, values, (size_t)argc / 2);
        }
        goto done;*/
   
    /* The rest are all parsed in the same way. */
    case RECEIVE_DATA:
    
        break;
    case SET_LINE_CODING:
        
        break;
    case SET_CONTROL_LINE_STATE:
        
        break;
    case SEND_BREAK:
        
        break;
    case SERIAL_STATE:
        
        break;
   
    }

    if (argc == 0) {
        fprintf(stderr, "You must specify at least one cpmmand.\n");
        rc = EXIT_FAILURE;
        goto done;
    }
   
 /*   printf("UARTs:");
    for (i = 0; i < argc; i++) {
        gpios[i] = (uint8_t)atoi(argv[2 + i]);
        printf(" %u", gpios[i]);
    }
    printf("\n");
    handler(gpios, (size_t)argc);*/
 done:
   
    if (rc) {
        usage(rc);
    }
    return 0;
#endif
}


/**
 * @brief The receive callback function.
 *
 * For verifying the driver can correctly call this callback.
 *
 * @param ls The present UART line status.
 * @return None.
 */
static void rx_callback(uint8_t *buffer, int length, int error)
{
	int i = 0;
    buffer[length] = 0x00; /* string end */
    //gb_info("%s(): length %d    \n", __func__,length);
    gb_info("%s(): data = %s, length = %d, error = %d    \n", __func__,buffer, length, error);
 	//gb_info("%s():   +++   length = %d \n", __func__,length );

	//for(i=0;i< length;i++)
		//gb_info("%s(): char[%d] = %c   \n", __func__,i,buffer[i]);
		
  
 


    
	switch (buffer[0]) {
  //   These are special cases. 
    case 'h':
        print_usage();
        break;
    case 'c':
        do_test_set_modem_ctrl(dev);
        break;
    case 'C':
        do_test_get_modem_ctrl(dev); //??
        break;
    case 'e':
        do_test_get_modem_status(dev);
        break;
    case 'l':
        do_test_get_line_status(dev);
        break;
    case 'B':
        do_test_set_break_on(dev);
        break;
    case 'b':
        do_test_set_break_off(dev);
        break;
    case 's':
        do_test_data_transmit(dev);
        break;
	default:
		gb_info("%s(): length %d    \n", __func__,length);
		break;
        
	}
    
  
      int ret, recv = 0;
    uint8_t rx_buf2[16];
   // memset(&rx_buf,0x00,sizeof(rx_buf));

    ret = device_uart_start_receiver(dev, rx_buf2, 16, NULL, NULL, rx_callback);
    if (ret) {
        //fprintf(stderr, "device_uart_start_receiver failed: %d\n", ret);
        gb_info("%s(): device_uart_start_receiver failed: %d \n", __func__,ret);
        
    }
    
 
    //do_test_data_receive(dev);    	
		
}
