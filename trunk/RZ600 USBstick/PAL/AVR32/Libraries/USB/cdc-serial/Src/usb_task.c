/* This source file is part of the ATMEL AVR32-SoftwareFramework-AT32UC3A3-0.7.0 Release */

/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file ******************************************************************
 *
 * \brief Management of the USB task either device/host or both.
 *
 * The USB task selects the correct USB task (USB device task or USB host
 * task) to be executed depending on the current mode available.
 *
 * According to the values of USB_DEVICE_FEATURE and USB_HOST_FEATURE
 * (located in the conf_usb.h file), the USB task can be configured to
 * support USB device mode or USB host mode or both for a dual-role device
 * application.
 *
 * This module also contains the general USB interrupt subroutine. This
 * subroutine is used to detect asynchronous USB events.
 *
 * Note:
 *   - The USB task belongs to main; the USB device and host tasks do not.
 *     They are called from the general USB task.
 *   - See the conf_usb.h file for more details about the configuration of
 *     this module.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR32
 * - Supported devices:  All AVR32 devices with a USB module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ***************************************************************************/

/* Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. ATMEL grants developer a non-exclusive, limited license to use the Software
 * as a development platform solely in connection with an Atmel AVR product
 * ("Atmel Product").
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 */
/**
 * $Id: usb_task.c 32382 2012-06-20 10:43:20Z yogesh.bellan $
 */
//_____  I N C L U D E S ___________________________________________________

#include "avr32types.h"
#include "compiler.h"
//#include "intc.h"

#include "conf_usb.h"
#include "usb_drv.h"
#include "usb_task.h"

#if USB_DEVICE_FEATURE == ENABLED
#include "usb_descriptors.h"
#include "usb_device_task.h"
#endif

#include "pal_internal.h"


//_____ M A C R O S ________________________________________________________


//_____ D E F I N I T I O N S ______________________________________________

//!
//! Public: U16 g_usb_event
//! usb_connected is used to store USB events detected upon
//! USB general interrupt subroutine
//! Its value is managed by the following macros (See \ref usb_task.h file)
//! Usb_send_event(x)
//! Usb_ack_event(x)
//! Is_usb_event(x)
//! Usb_clear_all_event()
volatile U16 g_usb_event = 0;


#if USB_DEVICE_FEATURE == ENABLED

//!
//! Public: Bool usb_connected
//! usb_connected is set to TRUE when VBus has been detected
//! usb_connected is set to FALSE otherwise
//! Used with USB_DEVICE_FEATURE == ENABLED only
extern volatile Bool usb_connected;

#endif


#if USB_HOST_FEATURE == ENABLED

static const char log_device_disconnected[] = "Device disconnected\n";

//!
//! Private: U8 private_sof_counter
//! Incremented  by host SOF interrupt subroutime
//! This counter is used to detect time-out in host requests.
//! It must not be modified by the user applicative tasks.
volatile U32 private_sof_counter;

#if USB_HOST_PIPE_INTERRUPT_TRANSFER == ENABLE
extern volatile Bool g_sav_int_sof_enable;
extern volatile S_pipe_int it_pipe_str[MAX_PEP_NB];
#endif

#endif


#if USB_DEVICE_FEATURE == ENABLED && USB_HOST_FEATURE == ENABLED

static const char log_pin_id_changed[]      = "Pin Id changed\n";

//!
//! Public: U8 g_usb_mode
//! Used in dual-role application (both device/host) to store
//! the current mode the USB controller is operating
volatile U8 g_usb_mode = USB_MODE_UNDEFINED;
static volatile U8 g_old_usb_mode;

#endif

//volatile U16 g_usb_event = 0;
#if (USB_HOST_FEATURE == ENABLED) && (USB_DEVICE_FEATURE == ENABLED) && (USB_HIGH_SPEED_SUPPORT==ENABLED)
static U8 private_sof_counter_HS = 0;  // Full speed SOF = 1ms , High speed uSOF = 125us
#endif

//_____ D E C L A R A T I O N S ____________________________________________




//! @brief This function initializes the USB process.
//!
//! Depending on the mode supported (HOST/DEVICE/DUAL_ROLE) the function
//! calls the coresponding USB mode initialization function
void usb_task_init(void)
{
    Usb_force_device_mode();
    usb_device_task_init();
}


void usb_task(void)
{
    usb_device_task();
    // ---- DUAL-ROLE DEVICE/HOST USB MODE -----------------------------------------
#if USB_DEVICE_FEATURE == ENABLED && USB_HOST_FEATURE == ENABLED
    if (g_old_usb_mode != g_usb_mode)
    {
        if (Is_usb_id_device())
        {
            usb_device_task_init();
        }
        else
        {
            private_sof_counter = 0;
            usb_host_task_init();
        }
        g_old_usb_mode = g_usb_mode;  // Store current USB mode, for mode change detection
        Usb_enable_id_interrupt();
        Enable_global_interrupt();
    }

    // Depending on current USB mode, launch the correct USB task (device or host)
    switch (g_old_usb_mode)
    {
        case USB_MODE_DEVICE:
            usb_device_task();
            break;
        case USB_MODE_HOST:
            usb_host_task();
            break;
        case USB_MODE_UNDEFINED:
        default:
            break;
    }
    // -----------------------------------------------------------------------------

    // ---- DEVICE-ONLY USB MODE ---------------------------------------------------
#elif USB_DEVICE_FEATURE == ENABLED
    usb_device_task();
    // -----------------------------------------------------------------------------

    // ---- REDUCED-HOST-ONLY USB MODE ---------------------------------------------
#elif USB_HOST_FEATURE == ENABLED
    usb_host_task();
    // -----------------------------------------------------------------------------

    // ---- ERROR, NO MODE ENABLED -------------------------------------------------
#else
#error At least one of USB_DEVICE_FEATURE and USB_HOST_FEATURE must be enabled
#endif
    // -----------------------------------------------------------------------------

}


//! @brief USB interrupt routine
//!
//! This function is called each time a USB interrupt occurs.
//! The following USB DEVICE events are taken in charge:
//! - VBus On / Off
//! - Start-of-Frame
//! - Suspend
//! - Wake-Up
//! - Resume
//! - Reset
//!
//! The following USB HOST events are taken in charge:
//! - Device connection
//! - Device Disconnection
//! - Start-of-Frame
//! - ID pin change
//! - SOF (or Keep alive in low-speed) sent
//! - Wake-up on USB line detected
//! - Pipe events
//!
//! For each event, the user can launch an action by completing the associated
//! \#define (see the conf_usb.h file to add actions on events).
//!
//! Note: Only interrupt events that are enabled are processed.
//!
//! Warning: If device and host tasks are not tasks in an RTOS, rough events
//! like ID transition, VBus transition, device disconnection, etc. that need to
//! kill then restart these tasks may lead to an undefined state if they occur
//! just before something is activated in the USB macro (e.g. pipe/endpoint
//! transfer...).
//!
//! @return Nothing in the standalone configuration; a boolean indicating
//!         whether a task switch is required in the FreeRTOS configuration
//#if (defined __GNUC__)
//  __attribute__((__interrupt__))
//#elif (defined __ICCAVR32__)
// // __interrupt static
//  #pragma handler = AVR32_USBB_IRQ_GROUP, 1
//__interrupt
//#endif

#if (defined __GNUC__)
__attribute__((__interrupt__))
#elif (defined __ICCAVR32__)
__interrupt
#endif
void usb_general_interrupt(void)
{
#if USB_HOST_FEATURE == ENABLED && USB_HOST_PIPE_INTERRUPT_TRANSFER == ENABLE
    U8 i;
#endif

    // ---------- DEVICE/HOST events management ------------------------------------
#if USB_DEVICE_FEATURE == ENABLED && USB_HOST_FEATURE == ENABLED
    // ID pin change detection
    if (Is_usb_id_transition() && Is_usb_id_interrupt_enabled())
    {
        g_usb_mode = (Is_usb_id_device()) ? USB_MODE_DEVICE : USB_MODE_HOST;
        Usb_ack_id_transition();
        if (g_usb_mode != g_old_usb_mode) // Basic debounce
        {
            // Previously in device mode, check if disconnection was detected
            if (g_old_usb_mode == USB_MODE_DEVICE)
            {
                if (usb_connected)
                {
                    // Device mode diconnection actions
                    usb_connected = FALSE;
                    usb_configuration_nb = 0;
                    Usb_vbus_off_action();
                }
            }
            // Previously in host mode, check if disconnection was detected
            else if (Is_host_attached())
            {
                // Host mode diconnection actions
                device_state = DEVICE_UNATTACHED;
                Host_device_disconnection_action();
            }
            LOG_STR(log_pin_id_changed);
            Usb_send_event((Is_usb_device()) ? EVT_USB_DEVICE_FUNCTION :
                           EVT_USB_HOST_FUNCTION);
            Usb_id_transition_action();
            //! @todo ID pin hot state change!!!
            // Preliminary management: HARDWARE RESET!!!
#if ID_PIN_CHANGE_GENERATE_RESET == ENABLE
            // Hot ID transition generates CPU reset
            Usb_disable();
            Usb_disable_otg_pad();
#endif
        }
    }
#endif  // End DEVICE/HOST FEATURE MODE

    // ---------- DEVICE events management -----------------------------------------
#if USB_DEVICE_FEATURE == ENABLED
#if USB_HOST_FEATURE == ENABLED
    // If both device and host features are enabled, check if device mode is engaged
    // (accessing the USB registers of a non-engaged mode, even with load operations,
    // may corrupt USB FIFO data).
    if (Is_usb_device())
#endif
    {
        // VBus state detection
        if (Is_usb_vbus_transition() && Is_usb_vbus_interrupt_enabled())
        {
            Usb_ack_vbus_transition();
            if (Is_usb_vbus_high())
            {
                usb_start_device();
                Usb_send_event(EVT_USB_POWERED);
                Usb_vbus_on_action();
            }
            else
            {
                Usb_unfreeze_clock();
                Usb_detach();
                usb_connected = FALSE;
                usb_configuration_nb = 0;
                Usb_send_event(EVT_USB_UNPOWERED);
                Usb_vbus_off_action();
            }
        }
        // Device Start-of-Frame received
        if (Is_usb_sof() && Is_usb_sof_interrupt_enabled())
        {
            Usb_ack_sof();
            Usb_sof_action();
        }
        // Device Suspend event (no more USB activity detected)
        if (Is_usb_suspend() && Is_usb_suspend_interrupt_enabled())
        {
            Usb_ack_suspend();
            Usb_enable_wake_up_interrupt();
            (void)Is_usb_wake_up_interrupt_enabled();
            Usb_freeze_clock();
            Usb_send_event(EVT_USB_SUSPEND);
            Usb_suspend_action();
        }
        // Wake-up event (USB activity detected): Used to resume
        if (Is_usb_wake_up() && Is_usb_wake_up_interrupt_enabled())
        {
            Usb_unfreeze_clock();
            (void)Is_usb_clock_frozen();
            Usb_ack_wake_up();
            Usb_disable_wake_up_interrupt();
            Usb_wake_up_action();
            Usb_send_event(EVT_USB_WAKE_UP);
        }
        // Resume state bus detection
        if (Is_usb_resume() && Is_usb_resume_interrupt_enabled())
        {
            Usb_disable_wake_up_interrupt();
            Usb_ack_resume();
            Usb_disable_resume_interrupt();
            Usb_resume_action();
            Usb_send_event(EVT_USB_RESUME);
        }
        // USB bus reset detection
        if (Is_usb_reset() && Is_usb_reset_interrupt_enabled())
        {
            Usb_ack_reset();
            usb_init_device();
            Usb_reset_action();
            Usb_send_event(EVT_USB_RESET);
        }
    }
#endif  // End DEVICE FEATURE MODE

    // ---------- HOST events management -------------------------------------------
#if USB_HOST_FEATURE == ENABLED
#if USB_DEVICE_FEATURE == ENABLED
    // If both device and host features are enabled, check if host mode is engaged
    // (accessing the USB registers of a non-engaged mode, even with load operations,
    // may corrupt USB FIFO data).
    else
#endif
    {
        // The device has been disconnected
        if (Is_host_device_disconnection() && Is_host_device_disconnection_interrupt_enabled())
        {
            host_disable_all_pipes();
            Host_ack_device_disconnection();
#if USB_HOST_PIPE_INTERRUPT_TRANSFER == ENABLE
            reset_it_pipe_str();
#endif
            device_state = DEVICE_UNATTACHED;
            LOG_STR(log_device_disconnected);
            Usb_send_event(EVT_HOST_DISCONNECTION);
            Host_device_disconnection_action();
#ifdef FREERTOS_USED
            // Release the semaphore in order to start a new device/host task
            taskENTER_CRITICAL();
            xSemaphoreGiveFromISR(usb_tsk_semphr, &task_woken);
            taskEXIT_CRITICAL();
#endif
        }
        // Device connection
        if (Is_host_device_connection() && Is_host_device_connection_interrupt_enabled())
        {
            Host_ack_device_connection();
            host_disable_all_pipes();
            Host_device_connection_action();
        }
        // Host Start-of-Frame has been sent
        if (Is_host_sof() && Is_host_sof_interrupt_enabled())
        {
            Host_ack_sof();
            Usb_send_event(EVT_HOST_SOF);
#if (USB_HIGH_SPEED_SUPPORT==ENABLED)
            if ( Is_usb_full_speed_mode() )
            {
                private_sof_counter++;
            }
            else
            {
                private_sof_counter_HS++;
                if ( 0 == (private_sof_counter_HS % 8) )
                {
                    private_sof_counter++;
                }
            }
#else
            private_sof_counter++;
#endif
            // Delay time-out management for interrupt tranfer mode in host mode
#if USB_HOST_PIPE_INTERRUPT_TRANSFER == ENABLE && TIMEOUT_DELAY_ENABLE == ENABLE
            if (private_sof_counter >= 250) // Count 250 ms (SOF @ 1 ms)
            {
                private_sof_counter = 0;
                for (i = 0; i < MAX_PEP_NB; i++)
                {
                    if (it_pipe_str[i].enable &&
                        ++it_pipe_str[i].timeout > TIMEOUT_DELAY && Host_get_pipe_type(i) != TYPE_INTERRUPT)
                    {
                        it_pipe_str[i].enable = FALSE;
                        it_pipe_str[i].status = PIPE_DELAY_TIMEOUT;
                        Host_reset_pipe(i);
                        if (!is_any_interrupt_pipe_active() && !g_sav_int_sof_enable) // If no more transfer is armed
                        {
                            Host_disable_sof_interrupt();
                        }
                        it_pipe_str[i].handler(PIPE_DELAY_TIMEOUT, it_pipe_str[i].nb_byte_processed);
                    }
                }
            }
#endif
            Host_sof_action();
        }
        // Host Wake-up has been received
        if (Is_host_hwup() && Is_host_hwup_interrupt_enabled())
        {
            // CAUTION: HWUP can be cleared only when USB clock is active (not frozen)!
            //! @todo Implement this on the silicon version
            //Pll_start_auto();               // First Restart the PLL for USB operation
            //Wait_pll_ready();               // Make sure PLL is locked
            Usb_unfreeze_clock();           // Enable clock on USB interface
            (void)Is_usb_clock_frozen();    // Make sure USB interface clock is enabled
            Host_disable_hwup_interrupt();  // Wake-up interrupt should be disabled as host is now awoken!
            Host_ack_hwup();                // Clear HWUP interrupt flag
            Usb_send_event(EVT_HOST_HWUP);  // Send software event
            Host_hwup_action();             // Map custom action
        }
#if USB_HOST_PIPE_INTERRUPT_TRANSFER == ENABLE
        // Host pipe interrupts
        while ((i = Host_get_interrupt_pipe_number()) < MAX_PEP_NB)
        {
            usb_pipe_interrupt(i);
        }
#endif
    }
#endif  // End HOST FEATURE MODE
}



void usb_suspend_action(void)
{
    Enable_global_interrupt();
    //! @todo Implement this on the silicon version
    //Enter_power_down_mode();  // For example...
}
/* EOF */


