/*
 * Copyright (c) 2015 Google, Inc.
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

#include <errno.h>
#include <debug.h>
#include <stdlib.h>
#include <queue.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/device.h>
#include <nuttx/device_uart.h>
#include <nuttx/util.h>
#include <nuttx/config.h>
#include <nuttx/greybus/types.h>
#include <nuttx/greybus/greybus.h>
#include <arch/tsb/unipro.h>
#include <apps/greybus-utils/utils.h>
#include <arch/byteorder.h>

#include "uart-gb.h"

//#include "uart_test.h"

#if 1

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
    fprintf(stderr, "ms_callback: (%u)\n", ms);
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
    fprintf(stderr, "ms_callback: (%u)\n", ls);
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
    buffer[length] = 0x00; /* string end */
    fprintf(stderr, "rx_callback:\n");
    fprintf(stderr, " data = %s, length = %d, error = %d\n",
            buffer, length, error);
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
    fprintf(stderr, "modem control = %x\n", modem_ctrl);
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
    fprintf(stderr, "modem status = %x\n", modem_status);
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
    fprintf(stderr, "line status = %x\n", line_status);
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

    ret = device_uart_start_receiver(dev, rx_buf, 16, NULL, &recv, NULL);
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
    //do_test_set_modem_ctrl(dev);
    //do_test_get_modem_ctrl(dev);
    //do_test_get_modem_status(dev);
    //do_test_get_line_status(dev);
    //do_test_set_break(dev);
    do_test_attach_ms_callback(dev);
    do_test_attach_ls_callback(dev);
    do_test_data_transmit(dev);
    //do_test_data_receive(dev);
    //do_test_data_loopback(dev);
}

/**
 * @brief The main test function.
 *
 * Since there is only one uart port in tsb bridge chip, so we use the jtag for
 * dump debug message without console. Only pick test functions from above
 * do_test_xxx functions.
 *
 * @param dev The device driver handler.
 * @return 0 for success, -errno for failures.
 */
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int uart_test_main(int argc, char *argv[])
#endif
{
    struct device *dev;
    int ret;

    fprintf(stderr, "uart_test_main start.\n");

    dev = device_open(DEVICE_TYPE_UART_HW, 0);
    if (!dev) {
        fprintf(stderr, "open failed\n");
        return -EIO;
    }

    ret = device_uart_set_configuration(dev, BAUD_115200, NO_PARITY, 8,
                                        ONE_STOP_BIT, 0);
    if (ret) {
        fprintf(stderr, "device_uart_set_configuration failed: %d\n", ret);
        goto err_device_opened;
    }
    
    ret = do_test_data_loopback(dev);
    if (ret) {
        fprintf(stderr, "uart_test_main failed: %d\n", ret);
    }

err_device_opened:
    device_close(dev);

    return 0;
}

#endif 



#define GB_UART_VERSION_MAJOR   0
#define GB_UART_VERSION_MINOR   1

/* Reserved operations for rx data buffer. */
#define MAX_RX_OPERATION        5
#define MAX_RX_BUF_SIZE         256

/* The id of error in protocol operating. */
#define GB_UART_EVENT_PROTOCOL_ERROR    1
#define GB_UART_EVENT_DEVICE_ERROR      2

/**
 * The buffer in operation structure.
 */
struct op_node {
    /** queue entry */
    sq_entry_t          entry;
    /** pointer to operation */
    struct gb_operation *operation;
    /** pointer to size of request in operation */
    uint16_t            *data_size;
    /** pointer to buffer of request in operation */
    uint8_t             *buffer;
};

/**
 * UART protocol information structure.
 */
struct gb_uart_info {
    /** cport from greybus */
    uint16_t            cport;
    /* status change */
    /** updated modem status from uart through callback */
    uint8_t             updated_ms;
    /** updated line status from uart through callback */
    uint8_t             updated_ls;
    /** the last status sent to the peer */
    uint16_t            last_serial_state;
    /** reserved operation for status report */
    struct gb_operation *ms_ls_operation;
    /** semaphore of status changing  */
    sem_t               status_sem;
    /** status change thread handle */
    pthread_t           status_thread;
    /* data receiving */
    /** available operation queue */
    sq_queue_t          free_queue;
    /** received data operation queue */
    sq_queue_t          data_queue;
    /** operation node in receiving */
    struct op_node      *rx_node;
    /** buffer size in operation */
    int                 rx_buf_size;
    /** amount of operations */
    int                 entries;
    /** flag for requesting a free operation in callback */
    int                 require_node;
    /** semaphore for notifying data received */
    sem_t               rx_sem;
    /** receiving data process threed */
    pthread_t           rx_thread;
    /** inform the thread should be terminated */
    int                 thread_stop;
    /** uart driver handle */
    struct device   *dev;
};

/* The structure for keeping protocol global data. */
static struct gb_uart_info *info = NULL;

/**
 * @brief Put the node to the back of the queue.
 *
 * @param queue The target queue to put.
 * @param node The pointer to node.
 * @return None.
 */
static void put_node_back(sq_queue_t *queue, struct op_node *node)
{
    irqstate_t flags = irqsave();

    sq_addlast(&node->entry, queue);

    irqrestore(flags);
}

/**
 * @brief Get a node from the queue.
 *
 * @param queue The target queue.
 * @return A pointer to the node or NULL for no node to get.
 */
static struct op_node *get_node_from(sq_queue_t *queue)
{
    struct op_node *node = NULL;
    irqstate_t flags = irqsave();
	//gb_info("%s(): GB uart  ++ \n", __func__);
    if (sq_empty(queue)) {
        irqrestore(flags);
        //gb_info("%s(): GB uart  null  - \n", __func__);
        return NULL;
    }
	
    node = (struct op_node *)sq_remfirst(queue);
    irqrestore(flags);
	//gb_info("%s(): GB uart  --  \n", __func__);
    return node;
}

/**
 * @brief Report the error.
 *
 * When error in callback or thread, function use this to report the error
 * instead of return error code.
 *
 * @param error The error id.
 * @return None.
 */
static void uart_report_error(int error, const char *func_name)
{
    /*
     * There should be event to report the error in callback and thread to the
     * peer, but it doesn't define in spec.
     */
    switch (error) {
    case GB_UART_EVENT_PROTOCOL_ERROR:
        gb_info("%s(): operation send error \n", func_name);
        break;
    case GB_UART_EVENT_DEVICE_ERROR:
        gb_info("%s(): device io error \n", func_name);
        break;
    default:
    break;
    }
}

/**
 * @brief Allocate operations for receiver buffers
 *
 * This function is allocating operation and use them as receiving buffers.
 *
 * @param max_nodes Maximum nodes.
 * @param buf_size Buffer size in operation.
 * @param queue Target queue.
 * @return 0 for success, -errno for failures.
 */
static int uart_alloc_op(int max_nodes, int buf_size, sq_queue_t *queue)
{
    struct gb_operation *operation = NULL;
    struct gb_uart_receive_data_request *request = NULL;
    struct op_node *node = NULL;
    int i = 0;

    for (i = 0; i < max_nodes; i++) {
        operation = gb_operation_create(info->cport,
                                        GB_UART_PROTOCOL_RECEIVE_DATA,
                                        sizeof(*request) + buf_size);
        if (!operation) {
            return -ENOMEM;
        }

        node = (struct op_node *)malloc(sizeof(struct op_node));
        if (!node) {
            return -ENOMEM;
        }
        node->operation = operation;

        request = gb_operation_get_request_payload(operation);
        node->data_size = &request->size;
        node->buffer = request->data;
        put_node_back(queue, node);
    }

    return 0;
}

/**
 * @brief Free operations
 *
 * This funciton destroy operations and node memory.
 *
 * @param queue Target queue.
 * @return None.
 */
static void uart_free_op(sq_queue_t *queue)
{
    struct op_node *node = NULL;

    node = get_node_from(queue);
    while (node) {
        gb_operation_destroy(node->operation);
        free(node);
        node = get_node_from(queue);
    }
}

/**
 * @brief Callback for modem status change
 *
 * Callback for device driver modem status changes. This function can be called
 * when device driver detect modem status changes.
 *
 * @param ms The updated modem status.
 * @return None.
 */
static void uart_ms_callback(uint8_t ms)
{
    info->updated_ms = ms;

    sem_post(&info->status_sem);
}

/**
 * @brief Callback for line status change
 *
 * Callback for device driver line status changes. This function can be called
 * when device driver detect line status changes.
 *
 * @param ls The updated modem status.
 * @return None.
 */
static void uart_ls_callback(uint8_t ls)
{
    info->updated_ls = ls;

    sem_post(&info->status_sem);
}

/**
 * @brief Callback for data receiving
 *
 * The callback function provided to device driver for being notified when
 * driver received a data stream.
 * It put the current operation to received queue and gets another operation to
 * continue receiving. Then notifies rx thread to process.
 *
 * @param buffer Data buffer.
 * @param length Received data length.
 * @param error Error code when driver receiving.
 * @return None.
 */
static void uart_rx_callback(uint8_t *buffer, int length, int error)
{
    struct op_node *node;
    int ret;

    *info->rx_node->data_size = cpu_to_le16(length);
    put_node_back(&info->data_queue, info->rx_node);
	
	int i=0; 
	
	gb_info("%s():   +++   length = %d \n", __func__,length );

	for(i=0;i< length;i++)
		gb_info("%s(): char[%d] = %c   \n", __func__,i,info->rx_node->buffer[i]);


    node = get_node_from(&info->free_queue);
    if (!node) {
        /*
         * there is no free buffer, inform the rx thread to engage another uart
         * receiver.
         */
         gb_info("%s():   +++   no free buffer \n", __func__ );
        info->require_node = 1;
        return;
    }

    info->rx_node = node;
    ret = device_uart_start_receiver(info->dev, node->buffer, info->rx_buf_size,
                                     NULL, NULL, uart_rx_callback);
    if (ret) {
		gb_info("%s():   +++   ret = %d \n", __func__,ret );
        uart_report_error(GB_UART_EVENT_PROTOCOL_ERROR, __func__);
    }

    sem_post(&info->rx_sem);
}

/**
 * @brief Parse the modem and line stauts
 *
 * This function parses the UART modem and line status to the bitmask of
 * protocol serial state.
 *
 * @param data The regular thread data.
 * @return The parsed value of protocol serial state bitmask.
 */
static uint16_t parse_ms_ls_registers(uint8_t modem_status, uint8_t line_status)
{
    uint16_t status = 0;

    if (modem_status & MSR_DCD) {
        status |= GB_UART_CTRL_DCD;
    }
    if (modem_status & MSR_DSR) {
        status |= GB_UART_CTRL_DSR;
    }
    if (modem_status & MSR_RI) {
        status |= GB_UART_CTRL_RI;
    }
    if (line_status & LSR_BI) {
        status |= GB_UART_CTRL_BRK;
    }
    if (line_status & LSR_FE) {
        status |= GB_UART_CTRL_FRAMING;
    }
    if (line_status & LSR_PE) {
        status |= GB_UART_CTRL_PARITY;
    }
    if (line_status & LSR_OE) {
        status |= GB_UART_CTRL_OVERRUN;
    }

    return status;
}

/**
 * @brief Modem and line status process thread
 *
 * This function is the thread for processing modem and line status change. It
 * uses the operation to send the event to the peer. It only sends the required
 * status for protocol, not the all status in UART.
 *
 * @param data The regular thread data.
 * @return None.
 */
static void *uart_status_thread(void *data)
{
    uint16_t updated_status = 0;
    struct gb_uart_serial_state_request *request;
    int ret = 0;

    while (1) {
        sem_wait(&info->status_sem);

        if (info->thread_stop) {
            break;
        }

        updated_status = parse_ms_ls_registers(info->updated_ms,
                                               info->updated_ls);
        /*
         * Only send the status bits which protocol need to know to peer
         */
        if (info->last_serial_state ^ updated_status) {
            info->last_serial_state = updated_status;
            request = gb_operation_get_request_payload(info->ms_ls_operation);
            request->control = 0; /* spec doesn't define what's in this field */
            request->data = cpu_to_le16(updated_status);
            ret = gb_operation_send_request(info->ms_ls_operation, NULL, false);
            if (ret) {
                uart_report_error(GB_UART_EVENT_PROTOCOL_ERROR, __func__);
            }
        }
    }

    return NULL;
}

/**
 * @brief Data receiving process thread
 *
 * This function is the thread for processing data receiving tasks. When
 * it wake up, it checks the receiving queue for processing the come in data.
 * If protocol is running out of operation, once it gets a free operation,
 * it passes to driver for continuing the receiving.
 *
 * @param data The regular thread data.
 * @return None.
 */
static void *uart_rx_thread(void *data)
{
    struct op_node *node = NULL;
    int ret;

	gb_info("%s(): GB uart  \n", __func__);
	
    while (1) {
		//gb_info("%s(): GB uart  rx_sem  + \n", __func__);
        sem_wait(&info->rx_sem);

		//gb_info("%s(): GB uart  rx_sem  - \n", __func__);
		
        if (info->thread_stop) {
            break;
        }

        node = get_node_from(&info->data_queue);
        //if (!node) {
        if (node) {
			gb_info("%s(): GB uart  node \n", __func__);
            ret = gb_operation_send_request(node->operation, NULL, false);
            if (ret) {
                uart_report_error(GB_UART_EVENT_PROTOCOL_ERROR, __func__);
            }
            put_node_back(&info->free_queue, node);
        }

        /*
         * In case there is no free node in callback.
         */
        if (info->require_node) {
            node = get_node_from(&info->free_queue);
            info->rx_node = node;
            gb_info("%s(): GB uart  device_uart_start_receiver \n", __func__);
            ret = device_uart_start_receiver(info->dev, node->buffer,
                                             info->rx_buf_size, NULL, NULL,
                                             uart_rx_callback);
            if (ret) {
                uart_report_error(GB_UART_EVENT_DEVICE_ERROR, __func__);
             //   gb_info("%s(): GB uart  device_uart_start_receiver  error \n", __func__);
            }
            info->require_node = 0;
        }
    }

    return NULL;
}

/**
 * @brief Releases resources for status change thread
 *
 * Terminates the thread for status change and releases the system resouces and
 * operations allocated by uart_status_cb_init().
 *
 * @param None.
 * @return None.
 */
static void uart_status_cb_deinit(void)
{
    if (info->status_thread != (pthread_t)0) {
        sem_post(&info->status_sem);
        pthread_join(info->status_thread, NULL);
    }

    sem_destroy(&info->status_sem);

    if (info->ms_ls_operation) {
        gb_operation_destroy(info->ms_ls_operation);
    }
}

/**
 * @brief Modem and line status event init process
 *
 * This function creates one operations and uses that request of operation
 * for sending the status change event to peer.
 *
 * @param None.
 * @return 0 on success, error code on failure.
 */
static int uart_status_cb_init(void)
{
    int ret;

    info->ms_ls_operation =
            gb_operation_create(info->cport,
                                GB_UART_PROTOCOL_SERIAL_STATE,
                                sizeof(struct gb_uart_serial_state_request));
    if (!info->ms_ls_operation) {
        return -ENOMEM;
    }

    ret = sem_init(&info->status_sem, 0, 0);
    if (ret) {
        return -ret;
    }

    ret = pthread_create(&info->status_thread, NULL, uart_status_thread, info);
    if (ret) {
        return -ret;
    }

    return 0;
}

/**
 * @brief Releases resources for receiver thread.
 *
 * Terminates the thread for receiver and releases the system resouces and
 * operations allocated by uart_receiver_cb_init().
 *
 * @param None.
 * @return None.
 */
static void uart_receiver_cb_deinit(void)
{
    if (info->rx_thread != (pthread_t)0) {
        sem_post(&info->rx_sem);
        pthread_join(info->rx_thread, NULL);
    }

    sem_destroy(&info->rx_sem);

    uart_free_op(&info->data_queue);
    uart_free_op(&info->free_queue);
}

/**
 * @brief Receiving data process initialization
 *
 * This function allocates OS resource to support the data receiving
 * function. It allocates two types of operations for undetermined length of
 * data. The semaphore works as message queue and all tasks are done in the
 * thread.
 *
 * @param None.
 * @return 0 for success, -errno for failures.
 */
static int uart_receiver_cb_init(void)
{
    int ret;

    sq_init(&info->free_queue);
    sq_init(&info->data_queue);

    info->entries = MAX_RX_OPERATION;
    info->rx_buf_size = MAX_RX_BUF_SIZE;

    ret = uart_alloc_op(info->entries, info->rx_buf_size, &info->free_queue);
    if (ret) {
       return ret;
    }

    ret = sem_init(&info->rx_sem, 0, 0);
    if (ret) {
        return ret;
    }

    ret = pthread_create(&info->rx_thread, NULL, uart_rx_thread, info);
    if (ret) {
        return ret;
    }

    return 0;
}

/**
 * @brief Protocol get version function.
 *
 * Returns the major and minor Greybus UART protocol version number supported
 * by the UART device.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_uart_protocol_version(struct gb_operation *operation)
{
    struct gb_uart_proto_version_response *response = NULL;

	//gb_info("%s():   +++   \n", __func__);

    response = gb_operation_alloc_response(operation, sizeof(*response));
    if (!response)
        return GB_OP_NO_MEMORY;

    response->major = GB_UART_VERSION_MAJOR;
    response->minor = GB_UART_VERSION_MINOR;
    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol send data function.
 *
 * Requests that the UART device begin transmitting characters. One or more
 * bytes to be transmitted will be supplied.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_uart_send_data(struct gb_operation *operation)
{
    int ret, size;
    int sent = 0;
    gb_info("%s():   +++   \n", __func__);
    struct gb_uart_send_data_request *request =
                    gb_operation_get_request_payload(operation);

    size = le16_to_cpu(request->size);
    /*
    ret = device_uart_start_transmitter(info->dev, request->data, size, NULL,
                                        &sent, NULL);
    if (ret) {
        return GB_OP_MALFUNCTION;
    }
	*/
    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol set line coding function.
 *
 * Sets the line settings of the UART to the specified baud rate, format,
 * parity, and data bits.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_uart_set_line_coding(struct gb_operation *operation)
{
    int ret;
    uint32_t baud;
    uint8_t parity, databits, stopbit;
    //gb_info("%s():   +++   \n", __func__);
    struct gb_serial_line_coding_request *request =
                    gb_operation_get_request_payload(operation);

    baud = le32_to_cpu(request->rate);

    switch (request->format) {
    case GB_SERIAL_1_STOP_BITS:
        stopbit = ONE_STOP_BIT;
        break;
    case GB_SERIAL_1_5_STOP_BITS:
        stopbit = ONE5_STOP_BITS;
        break;
    case GB_SERIAL_2_STOP_BITS:
        stopbit = TWO_STOP_BITS;
        break;
    default:
        return GB_OP_INVALID;
        break;
    }

    switch (request->parity) {
    case GB_SERIAL_NO_PARITY:
        parity = NO_PARITY;
        break;
    case GB_SERIAL_ODD_PARITY:
        parity = ODD_PARITY;
        break;
    case GB_SERIAL_EVEN_PARITY:
        parity = EVEN_PARITY;
        break;
    case GB_SERIAL_MARK_PARITY:
        parity = MARK_PARITY;
        break;
    case GB_SERIAL_SPACE_PARITY:
        parity = SPACE_PARITY;
        break;
    default:
        return GB_OP_INVALID;
        break;
    }

    if (request->data > 8 || request->data < 5) {
        return GB_OP_INVALID;
    }

    databits = request->data;

    ret = device_uart_set_configuration(info->dev, baud, parity, databits,
                                        stopbit, 0); /* flow is 0 */
    if (ret) {
        return GB_OP_MALFUNCTION;
    }

    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol set RTS & DTR line status function.
 *
 * Controls RTS and DTR line states of the UART.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_uart_set_control_line_state(struct gb_operation *operation)
{
    int ret;
    uint8_t modem_ctrl = 0;
	uint16_t control;
    //gb_info("%s():   +++   \n", __func__);
    struct gb_uart_set_control_line_state_request *request =
                gb_operation_get_request_payload(operation);

    ret = device_uart_get_modem_ctrl(info->dev, &modem_ctrl);
    if (ret) {
        return GB_OP_MALFUNCTION;
    }

    control = le16_to_cpu(request->control);
    if (control & GB_UART_CTRL_DTR) {
        modem_ctrl |= MCR_DTR;
    } else {
        modem_ctrl &= ~MCR_DTR;
    }

    if (control & GB_UART_CTRL_RTS) {
        modem_ctrl |= MCR_RTS;
    } else {
        modem_ctrl &= ~MCR_RTS;
    }

    ret = device_uart_set_modem_ctrl(info->dev, &modem_ctrl);
    if (ret) {
        return GB_OP_MALFUNCTION;
    }

    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol send break function.
 *
 * Requests that the UART generate a break condition on its transmit line.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static uint8_t gb_uart_send_break(struct gb_operation *operation)
{
    int ret;
    //gb_info("%s():   +++   \n", __func__);
    struct gb_uart_set_break_request *request =
                  gb_operation_get_request_payload(operation);

    ret = device_uart_set_break(info->dev, request->state);
    if (ret) {
        return GB_OP_MALFUNCTION;
    }

    return GB_OP_SUCCESS;
}

/**
 * @brief Protocol initialization function.
 *
 * This function perform the protocto initialization function, such as open
 * the cooperation device driver, launch threads, create buffers etc.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return GB_OP_SUCCESS on success, error code on failure.
 */
static int gb_uart_init(unsigned int cport)
{
    int ret;
    uint8_t ms = 0, ls = 0;

    info = zalloc(sizeof(*info));
    if (info == NULL) {
        return -ENOMEM;
    }

    gb_info("%s(): GB uart info struct: 0x%08p \n", __func__, info);

    info->cport = cport;

    ret = uart_status_cb_init();
 //   gb_info("%s(): GB uart uart_status_cb_init: %d \n", __func__, ret);
    if (ret) {
        goto err_free_info;
    }

	

    ret = uart_receiver_cb_init();
    //gb_info("%s(): GB uart uart_status_cb_init: %d \n", __func__, ret);
    if (ret) {
        goto err_status_cb_init;
    }

    info->dev = device_open(DEVICE_TYPE_UART_HW, 0);
    //gb_info("%s(): GB uart device_open: %d \n", __func__, ret);
    if (!info->dev) {
	//	gb_info("%s(): GB uart device_open: null \n", __func__);
        goto err_receiver_cb_init;
    }

    /* update serial status */
    // gb_info("%s(): GB uart device_uart_get_modem_status: ++ \n", __func__);
    ret = device_uart_get_modem_status(info->dev, &ms);
   // gb_info("%s(): GB uart device_uart_get_modem_status: %d \n", __func__, ret);
    if (ret) {
        goto err_receiver_cb_init;
    }

    ret = device_uart_get_line_status(info->dev, &ls);
    //gb_info("%s(): GB uart device_uart_get_line_status: %d \n", __func__, ret);
    if (ret) {
        goto err_receiver_cb_init;
    }

    info->last_serial_state = parse_ms_ls_registers(ms, ls);

    ret = device_uart_attach_ms_callback(info->dev, uart_ms_callback);
   // gb_info("%s(): GB uart device_uart_attach_ms_callback: %d \n", __func__, ret);
    if (ret) {
        goto err_receiver_cb_init;
    }

    ret = device_uart_attach_ls_callback(info->dev, uart_ls_callback);
    //gb_info("%s(): GB uart device_uart_attach_ls_callback: %d \n", __func__, ret);
    if (ret) {
        goto err_receiver_cb_init;
    }

    /* trigger the first receiving */
    info->require_node = 1;
    sem_post(&info->rx_sem);



	 //test 
	 /*
	 gb_info("%s(): GB uart info struct:send_break \n", __func__);
	 device_uart_set_break(info->dev, 0);
	*/
	
	/*
	gb_info("%s(): GB uart info struct:send_data \n", __func__);
	u8 test[] = "12345678"; 
	device_uart_start_transmitter(info->dev,
							test, sizeof(test), NULL,
							NULL, NULL);
	
	
	    int aret, xmit = 0;
    
    uint8_t tx_buf[] = "transmit_test";

    aret = device_uart_start_transmitter(info->dev, tx_buf, sizeof(tx_buf), NULL,
                                        &xmit, NULL);
    if (aret) {
        fprintf(stderr, "device_uart_start_transmitter failed: %d\n", aret);
        return aret;
    }

    fprintf(stderr, "device_uart_start_transmitter: sent = %d\n", xmit);
	
	*/
	
	/*
	 gb_info("%s(): GB uart info struct:send_break \n", __func__);
	 device_uart_set_break(info->dev, 0);
	*/

	 gb_info("%s(): GB uart  ---  \n", __func__);
	 
	 
	 
	
	 {
		 ret = device_uart_set_configuration(info->dev, BAUD_115200, NO_PARITY, 8,
                                        ONE_STOP_BIT, 0);
		if (ret) {
			fprintf(stderr, "device_uart_set_configuration failed: %d\n", ret);
			
		}
	  do_all_test(info->dev);
	 }
	 
    return 0;

err_receiver_cb_init:
    uart_receiver_cb_deinit();
err_status_cb_init:
    uart_status_cb_deinit();
err_free_info:
    free(info);

    return ret;
}

/**
 * @brief Protocol exit function.
 *
 * This function can be called when protocol terminated.
 *
 * @param operation The pointer to structure of gb_operation.
 * @return None.
 */
static void gb_uart_exit(unsigned int cport)
{
    device_uart_attach_ms_callback(info->dev, NULL);

    device_uart_attach_ls_callback(info->dev, NULL);

    uart_status_cb_deinit();

    uart_receiver_cb_deinit();

    device_close(info->dev);

    free(info);
}

static struct gb_operation_handler gb_uart_handlers[] = {
    GB_HANDLER(GB_UART_PROTOCOL_VERSION, gb_uart_protocol_version),
    GB_HANDLER(GB_UART_PROTOCOL_SEND_DATA, gb_uart_send_data),
    GB_HANDLER(GB_UART_PROTOCOL_SET_LINE_CODING, gb_uart_set_line_coding),
    GB_HANDLER(GB_UART_PROTOCOL_SET_CONTROL_LINE_STATE,
               gb_uart_set_control_line_state),
    GB_HANDLER(GB_UART_PROTOCOL_SEND_BREAK, gb_uart_send_break),
};

struct gb_driver uart_driver = {
    .init = gb_uart_init,
    .exit = gb_uart_exit,
    .op_handlers = (struct gb_operation_handler*) gb_uart_handlers,
    .op_handlers_count = ARRAY_SIZE(gb_uart_handlers),
};

/**
 * @brief Protocol registering function.
 *
 * This function can be called by greybus to register the UART protocol.
 *
 * @param cport The number of CPort.
 * @return None.
 */
void gb_uart_register(int cport)
{
    gb_info("%s(): cport %d \n", __func__, cport);
    gb_register_driver(cport, &uart_driver);
}

