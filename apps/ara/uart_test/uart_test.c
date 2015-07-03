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
   // fprintf(stdout, "ms_callback: (%u)\n", ms);
   //printf("ms_callback %x\n", ms);
   //printf("ms_callback \n");
   
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
  //  fprintf(stdout, "ls_callback: (%u)\n", ls);
  // printf("ls_callback %x \n",ls);
   //printf("ls_callback \n");
   
   gb_info("%s():   +++ %x    \n", __func__,ls);
     
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
    gb_info("%s():   +++  length %d    \n", __func__,length);
    gb_info("%s():   +++ data = %s, length = %d, error = %d    \n", __func__,buffer, length, error);
    
   // fprintf(stderr, "rx_callback: length %d \n", length);
   // fprintf(stdout, " data = %s, length = %d, error = %d\n",
   //         buffer, length, error);
   
	//for(i=0;i< length;i++)
		//fprintf(stdout, "rx_callback: char[%d] = %c \n", i,buffer[i]);
		
		
		
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
    fprintf(stderr, "tx_callback:\n");
    fprintf(stderr, " data = %s, length = %d, error = %d\n",
            buffer, length, error);
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
    fprintf(stderr, "set modem control = %u\n", modem_ctrl);
    ret = device_uart_set_modem_ctrl(dev, &modem_ctrl);
    if (ret) {
        fprintf(stderr, "device_uart_set_modem_ctrl failed: %d \n", ret);
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
        fprintf(stderr, "device_uart_get_modem_status failed: %d\n", ret);
        return ret;
    }
    fprintf(stderr, "get modem control = %u\n", modem_ctrl);
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
        fprintf(stderr, "device_uart_get_modem_status failed: %d\n", ret);
        return ret;
    }
    fprintf(stderr, "get modem status = %u\n", modem_status);
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
        fprintf(stderr, "do_test_get_line_status failed: %d\n", ret);
        return ret;
    }
    fprintf(stderr, "get line status = %x\n", line_status);
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
static int do_test_set_break(struct device *dev)
{
    int ret;

    ret = device_uart_set_break(dev, 1);
    if (ret) {
        fprintf(stderr, "device_uart_set_break 1 failed: %d\n", ret);
        return ret;
    }

    ret = device_uart_set_break(dev, 0);
    if (ret) {
        fprintf(stderr, "device_uart_set_break 0 failed: %d\n", ret);
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
        fprintf(stderr, "device_uart_attach_ms_callback NULL failed: %d \n",
               ret);
        return ret;
    }

    ret = device_uart_attach_ms_callback(dev, ms_callback);
    if (ret) {
        fprintf(stderr, "device_uart_attach_ms_callback with callback: %d\n",
               ret);
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
        fprintf(stderr, "device_uart_attach_ms_callback NULL failed: %d \n",
               ret);
        return ret;
    }

    ret = device_uart_attach_ls_callback(dev, ls_callback);
    if (ret) {
        fprintf(stderr, "device_uart_attach_ms_callback with callback: %d\n",
               ret);
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

	fprintf(stderr, "device_uart_start_transmitter ++: \n" );
	
    ret = device_uart_start_transmitter(dev, tx_buf, sizeof(tx_buf), NULL,
                                        &xmit, NULL);
    if (ret) {
        fprintf(stderr, "device_uart_start_transmitter failed: %d\n", ret);
        return ret;
    }

    fprintf(stderr, "device_uart_start_transmitter: sent = %d\n", xmit);

    return 0;
}

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

    ret = device_uart_start_receiver(dev, rx_buf, 16, NULL, &recv, rx_callback);
    if (ret) {
        fprintf(stderr, "device_uart_start_receiver failed: %d\n", ret);
        return ret;
    }

    rx_buf[recv] = 0x00; /* string end */
    fprintf(stderr, "device_uart_start_receiver: data = %s, recv = %d",
            rx_buf, recv);

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
        fprintf(stderr, "device_uart_start_receiver failed: %d\n", ret);
        return ret;
    }

    ret = device_uart_start_transmitter(dev, tx_buf, sizeof(tx_buf), NULL,
                                        NULL, &tx_callback);
    if (!ret) {
        fprintf(stderr, "device_uart_start_transmitter failed: %d\n", ret);
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
    PROTOCOL_VERSION,
    SEND_DATA,
    RECEIVE_DATA,
    SET_LINE_CODING,
    SET_CONTROL_LINE_STATE,
    SEND_BREAK,
    SERIAL_STATE,
    MAX_CMD,
};

struct command {
    const char shortc;
    const char *longc;
    const char *argspec;
    const char *help;
};

static const struct command commands[] = {
    [HELP]					= {'h', "help", NULL, "Print this message and exit"},
    [PROTOCOL_VERSION]		= {'p', "protocol-version", NULL, "Print number of protocol version\n" },
    [SEND_DATA]				= {'s', "send-data", "<uart> [...]", "Send data to UART"},
    [RECEIVE_DATA]			= {'r', "receive-data","<uart> [...]", "receive data from UART"},
    [SET_LINE_CODING]		= {'l', "set-line-coding", "<uart> [...]","set line coding to UART"},
    [SET_CONTROL_LINE_STATE]= {'c', "set-control-state", "<uart> [...]","set control line to UART"},
    [SEND_BREAK]			= {'b', "send-break", "<uart> [...]", "send break to UART"},
    [SERIAL_STATE]			= {'e', "serial-state", "<uart> [...]", "Get serial state value"},
};


static void print_usage(void)
{
    int i;
    printf("uart: usage:\n");
    for (i = 0; i < MAX_CMD; i++) {
        const char *argspec = commands[i].argspec;
        const char *space = " ";
        if (!argspec) {
            space = "";
            argspec = "";
        }
        printf(" uart_test [%c|%s]%s%s: %s\n",
               commands[i].shortc, commands[i].longc,
               space, argspec,
               commands[i].help);
    }
    printf("\n"
           "<uart> values range from 0 to the line count minus one.\n");
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



#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int uart_test_main(int argc, char *argv[])
#endif
{
#if 1
    struct device *dev;
    int ret;
   
  
   
    //fprintf(stderr, "hello uart_test_main start.\n");


	gb_info("%s():  hello  +++   \n", __func__);

	//usleep(50*1000);
	//fprintf(stderr, "hello device_open start.\n");
    dev = device_open(DEVICE_TYPE_UART_HW, 0);
    if (dev == NULL) {
        fprintf(stderr, "open failed\n");
        return -EIO;
    }

    ret = device_uart_set_configuration(dev, BAUD_115200, NO_PARITY, 8,
                                        ONE_STOP_BIT, 0);
    if (ret) {
        fprintf(stderr, "device_uart_set_configuration failed: %d\n", ret);
        goto err_device_opened;
    }
    
    
    
    do_all_test(dev);
    /*
    ret = do_test_data_loopback(dev);
    if (ret) {
        fprintf(stderr, "uart_test_main failed: %d\n", ret);
    }
	*/

err_device_opened:
    device_close(dev);

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
