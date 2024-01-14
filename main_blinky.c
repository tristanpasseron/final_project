/*
 * FreeRTOS V202107.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/******************************************************************************
 * NOTE 1: The FreeRTOS demo threads will not be running continuously, so
 * do not expect to get real time behaviour from the FreeRTOS Linux port, or
 * this demo application.  Also, the timing information in the FreeRTOS+Trace
 * logs have no meaningful units.  See the documentation page for the Linux
 * port for further information:
 * https://freertos.org/FreeRTOS-simulator-for-Linux.html
 *
 * NOTE 2:  This project provides two demo applications.  A simple blinky style
 * project, and a more comprehensive test and demo application.  The
 * mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting in main.c is used to select
 * between the two.  See the notes on using mainCREATE_SIMPLE_BLINKY_DEMO_ONLY
 * in main.c.  This file implements the simply blinky version.  Console output
 * is used in place of the normal LED toggling.
 *
 * NOTE 3:  This file only contains the source code that is specific to the
 * basic demo.  Generic functions, such FreeRTOS hook functions, are defined
 * in main.c.
 ******************************************************************************
 *
 * main_blinky() creates one queue, one software timer, and two tasks.  It then
 * starts the scheduler.
 *
 * The Queue Send Task:
 * The queue send task is implemented by the prvQueueSendTask() function in
 * this file.  It uses vTaskDelayUntil() to create a periodic task that sends
 * the value 100 to the queue every 200 milliseconds (please read the notes
 * above regarding the accuracy of timing under Linux).
 *
 * The Queue Send Software Timer:
 * The timer is an auto-reload timer with a period of two seconds.  The timer's
 * callback function writes the value 200 to the queue.  The callback function
 * is implemented by prvQueueSendTimerCallback() within this file.
 *
 * The Queue Receive Task:
 * The queue receive task is implemented by the prvQueueReceiveTask() function
 * in this file.  prvQueueReceiveTask() waits for data to arrive on the queue.
 * When data is received, the task checks the value of the data, then outputs a
 * message to indicate if the data came from the queue send task or the queue
 * send software timer.
 *
 * Expected Behaviour:
 * - The queue send task writes to the queue every 200ms, so every 200ms the
 *   queue receive task will output a message indicating that data was received
 *   on the queue from the queue send task.
 * - The queue send software timer has a period of two seconds, and is reset
 *   each time a key is pressed.  So if two seconds expire without a key being
 *   pressed then the queue receive task will output a message indicating that
 *   data was received on the queue from the queue send software timer.
 *
 * NOTE:  Console input and output relies on Linux system calls, which can
 * interfere with the execution of the FreeRTOS Linux port. This demo only
 * uses Linux system call occasionally. Heavier use of Linux system calls
 * may crash the port.
 */

#include <stdio.h>
#include <pthread.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Local includes. */
#include "console.h"

/* Priorities at which the tasks are created. */
/*#define mainQUEUE_RECEIVE_TASK_PRIORITY    ( tskIDLE_PRIORITY + 1 )
#define mainQUEUE_RECEIVE_TASK2_PRIORITY    ( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_RECEIVE_TASK3_PRIORITY    ( tskIDLE_PRIORITY + 3 )*/

#define TEMP   (tskIDLE_PRIORITY + 2 )
#define PRINT  (tskIDLE_PRIORITY + 4 )
#define CALCUL (tskIDLE_PRIORITY + 1 )
#define SEARCH (tskIDLE_PRIORITY + 3 )


/* The rate at which data is sent to the queue.  The times are converted from
 * milliseconds to ticks using the pdMS_TO_TICKS() macro. */
/*#define mainTASK_SEND_FREQUENCY_MS         pdMS_TO_TICKS( 100UL )
#define mainTIMER_SEND_FREQUENCY_MS        pdMS_TO_TICKS( 100UL )
#define mainTASK2_SEND_FREQUENCY_MS        pdMS_TO_TICKS( 100UL )
#define mainTASK3_SEND_FREQUENCY_MS        pdMS_TO_TICKS( 100UL )*/

#define TEMP_FREQUENCY    pdMS_TO_TICKS ( 1000UL )
#define PRINT_FREQUENCY    pdMS_TO_TICKS ( 1500UL )
#define CALCUL_FREQUENCY  pdMS_TO_TICKS  (2000UL )
#define SEARCH_FREQUENCY  pdMS_TO_TICKS  ( 1500UL)
#define main_FREQUENCY_MS  pdMS_TO_TICKS ( 3000UL )


/* The number of items the queue can hold at once. */
#define mainQUEUE_LENGTH                   ( 3 )

/* The values sent to the queue receive task from the queue send task and the
 * queue send software timer respectively. */
#define mainVALUE_SENT_FROM_TASK           ( 100UL )
#define mainVALUE_SENT_FROM_TIMER          ( 200UL )


/*-----------------------------------------------------------*/

/*
 * The tasks as described in the comments at the top of this file.
 */
/*static void prvQueueReceiveTask( void * pvParameters );
static void prvQueueReceiveTask2( void * pvParameters );
static void prvQueueReceiveTask3( void * pvParameters );
static void prvQueueSendTask( void * pvParameters );
static void prvQueueSendTask2( void * pvParameters );
static void prvQueueSendTask3( void * pvParameters );*/

static void vTemp( void );
static void vPrinter( void );
static void vCalcul( void );
static void vSearch( void );
static TimerHandle_t xTimer = NULL;
static QueueHandle_t xQueue = NULL;

/*
 * The callback function executed when the software timer expires.
 */
static void prvQueueSendTimerCallback( TimerHandle_t xTimerHandle );

/*-----------------------------------------------------------*/

/* The queue used by both tasks. */
//static QueueHandle_t xQueue = NULL;

/* A software timer that is started from the tick hook. */
//static TimerHandle_t xTimer = NULL;

/*-----------------------------------------------------------*/

/*** SEE THE COMMENTS AT THE TOP OF THIS FILE ***/
void main_blinky( void )
{
    const TickType_t xTimerPeriod = main_FREQUENCY_MS;

    /* Create the queue. */
    xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint32_t ) );

    if( xQueue != NULL )
    {
        /* Start the two tasks as described in the comments at the top of this
         * file. */
        xTaskCreate( vTemp,             /* The function that implements the task. */
                     "Temp",                            /* The text name assigned to the task - for debug only as it is not used by the kernel. */
                     configMINIMAL_STACK_SIZE,        /* The size of the stack to allocate to the task. */
                     NULL,                            /* The parameter passed to the task - not used in this simple case. */
                     TEMP, /* The priority assigned to the task. */
                     NULL );                          /* The task handle is not required, so NULL is passed. */
                     
        xTaskCreate( vPrinter,             /* The function that implements the task. */
                     "Print",                            /* The text name assigned to the task - for debug only as it is not used by the kernel. */
                     configMINIMAL_STACK_SIZE,        /* The size of the stack to allocate to the task. */
                     NULL,                            /* The parameter passed to the task - not used in this simple case. */
                     PRINT, /* The priority assigned to the task. */
                     NULL );
        
        xTaskCreate( vCalcul,             /* The function that implements the task. */
                     "Calcul",                            /* The text name assigned to the task - for debug only as it is not used by the kernel. */
                     configMINIMAL_STACK_SIZE,        /* The size of the stack to allocate to the task. */
                     NULL,                            /* The parameter passed to the task - not used in this simple case. */
                     PRINT, /* The priority assigned to the task. */
                     NULL );
                     
        xTaskCreate( vSearch,             /* The function that implements the task. */
                     "Search",                            /* The text name assigned to the task - for debug only as it is not used by the kernel. */
                     configMINIMAL_STACK_SIZE,        /* The size of the stack to allocate to the task. */
                     NULL,                            /* The parameter passed to the task - not used in this simple case. */
                     SEARCH, /* The priority assigned to the task. */
                     NULL );
        
        /*xTaskCreate( prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL );
        xTaskCreate( prvQueueSendTask2, "TX2", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK2_PRIORITY, NULL );
        xTaskCreate( prvQueueSendTask2, "TX3", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK3_PRIORITY, NULL );*/
        

        /* Create the software timer, but don't start it yet. */
        xTimer = xTimerCreate( "Timer",                     /* The text name assigned to the software timer - for debug only as it is not used by the kernel. */
                               xTimerPeriod,                /* The period of the software timer in ticks. */
                               pdTRUE,                      /* xAutoReload is set to pdTRUE. */
                               NULL,                        /* The timer's ID is not used. */
                               prvQueueSendTimerCallback ); /* The function executed when the timer expires. */

        if( xTimer != NULL )
        {
            xTimerStart( xTimer, 0 );
        }

        /* Start the tasks and timer running. */
        vTaskStartScheduler();
    }

    /* If all is well, the scheduler will now be running, and the following
     * line will never be reached.  If the following line does execute, then
     * there was insufficient FreeRTOS heap memory available for the idle and/or
     * timer tasks	to be created.  See the memory management section on the
     * FreeRTOS web site for more details. */
    for( ; ; )
    {
    }
}
/*-----------------------------------------------------------*/

void vTemp(void)
{
    clock_t start, end;
    const TickType_t xBlockTime = TEMP_FREQUENCY;
    float F = 118, C;
    double EXECUTION_LIMIT = 50;
    /* Initialise xNextWakeTime - this only needs to be done once. */
    TickType_t xNextWakeTime = xTaskGetTickCount();

    for (;;)
    {
        start = clock();  // Enregistrez le temps de début

        /* Place this task in the blocked state until it is time to run again.
         * The block time is specified in ticks, pdMS_TO_TICKS() was used to
         * convert a time specified in milliseconds into a time specified in ticks.
         * While in the Blocked state this task will not consume any CPU time. */
        vTaskDelayUntil(&xNextWakeTime, xBlockTime);

        C = (F - 32) / 1.8;

        printf("Temp: %f\n", C);

        end = clock();  // Enregistrez le temps de fin

        // Calculer le temps d'exécution en millisecondes
        double executionTime = ((double)(end - start) / CLOCKS_PER_SEC) * 1000;

        

        // Vérifier si le temps d'exécution est supérieur à la limite
        if (executionTime > EXECUTION_LIMIT)
        {
            const char *taskName1 = "vtemp";
            printf("La tâche %s n'est pas ordonnancée. Temps d'exécution réel: %f ms au lieu de %f ms.\n", taskName1, executionTime, EXECUTION_LIMIT);
            vTaskEndScheduler();  // Arrêter le scheduler FreeRTOS
        }
    }
}





void vPrinter(void)
{
    clock_t start, end;
    const TickType_t xBlockTime = PRINT_FREQUENCY;
    double EXECUTION_LIMIT = 60;

    /* Initialise xNextWakeTime - this only needs to be done once. */
    TickType_t xNextWakeTime = xTaskGetTickCount();

    for (;;)
    {
        start = clock();  // Enregistrez le temps de début

        /* Place this task in the blocked state until it is time to run again.
         * The block time is specified in ticks, pdMS_TO_TICKS() was used to
         * convert a time specified in milliseconds into a time specified in ticks.
         * While in the Blocked state this task will not consume any CPU time. */
        vTaskDelayUntil(&xNextWakeTime, xBlockTime);

        printf("tout fonctionne normalement""\n");

        end = clock();  // Enregistrez le temps de fin

        // Calculer le temps d'exécution en millisecondes
        double executionTime = ((double)(end - start) / CLOCKS_PER_SEC) * 1000;

        printf("Execution Time: %f ms\n", executionTime);

        // Vérifier si le temps d'exécution est supérieur à 80 ms
        if (executionTime > EXECUTION_LIMIT)
        {
            const char *taskName2 = "vPrinter";
            printf("La tâche %s n'est pas ordonnancée. Temps d'exécution réel: %f ms au lieu de %f ms.\n", taskName2, executionTime, EXECUTION_LIMIT);
            vTaskEndScheduler();  // Arrêter le scheduler FreeRTOS
        }
    }
}


void vCalcul(void)
{
    clock_t start, end;
    const TickType_t xBlockTime = CALCUL_FREQUENCY;
    double EXECUTION_LIMIT = 80;
	
    /* Initialise xNextWakeTime - this only needs to be done once. */
    TickType_t xNextWakeTime = xTaskGetTickCount();

    for (;;)
    {
        start = clock();  // Enregistrez le temps de début

        /* Place this task in the blocked state until it is time to run again.
         * The block time is specified in ticks, pdMS_TO_TICKS() was used to
         * convert a time specified in milliseconds into a time specified in ticks.
         * While in the Blocked state this task will not consume any CPU time. */
        vTaskDelayUntil(&xNextWakeTime, xBlockTime);

        /* Définissez deux nombres longs et multipliez-les */
        long int num1 = 123456789;
        long int num2 = 987654321;
        long int result = num1 * num2;

        /* Imprimez le résultat */
        printf("Multiplication result: %ld\n", result);

        end = clock();  // Enregistrez le temps de fin

        // Calculer le temps d'exécution en millisecondes
        double executionTime = ((double)(end - start) / CLOCKS_PER_SEC) * 1000;

        printf("Execution Time: %f ms\n", executionTime);

        // Vérifier si le temps d'exécution est supérieur à 80 ms
        if (executionTime > EXECUTION_LIMIT)
        {
            const char *taskName3 = "vCalcul";  // Changez le nom de la tâche ici
            printf("La tâche %s n'est pas ordonnancée. Temps d'exécution réel: %f ms au lieu de %f ms.\n", taskName3, executionTime, EXECUTION_LIMIT);
            vTaskEndScheduler();  // Arrêter le scheduler FreeRTOS
        }
    }
}



#define LIST_SIZE 50
#define ELEMENT_TO_SEARCH 42

void vSearch(void)
{
    clock_t start, end;
    const TickType_t xBlockTime = SEARCH_FREQUENCY;
    double EXECUTION_LIMIT = 55;

    /* Fixed and sorted list of 50 elements for binary search */
    int list[LIST_SIZE] = {
        1, 2, 4, 6, 8, 10, 12, 14, 16, 18,
        20, 22, 24, 26, 28, 30, 32, 34, 36, 38,
        40, 42, 44, 46, 48, 50, 52, 54, 56, 58,
        60, 62, 64, 66, 68, 70, 72, 74, 76, 78,
        80, 82, 84, 86, 88, 90, 92, 94, 96, 98
    };

    /* Initialise xNextWakeTime - this only needs to be done once. */
    TickType_t xNextWakeTime = xTaskGetTickCount();

    for (;;)
    {
        start = clock();  // Enregistrez le temps de début

        /* Place this task in the blocked state until it is time to run again.
         * The block time is specified in ticks, pdMS_TO_TICKS() was used to
         * convert a time specified in milliseconds into a time specified in ticks.
         * While in the Blocked state, this task will not consume any CPU time. */
        vTaskDelayUntil(&xNextWakeTime, xBlockTime);

        /* Binary search for the element in the list */
        int low = 0, high = LIST_SIZE - 1;
        int found = 0;

        while (low <= high)
        {
            int mid = (low + high) / 2;

            if (list[mid] == ELEMENT_TO_SEARCH)
            {
                printf("Element %d found at index %d\n", ELEMENT_TO_SEARCH, mid);
                found = 1;
                break;
            }
            else if (list[mid] < ELEMENT_TO_SEARCH)
            {
                low = mid + 1;
            }
            else
            {
                high = mid - 1;
            }
        }

        if (!found)
        {
            printf("Element %d not found in the list\n", ELEMENT_TO_SEARCH);
        }

        end = clock();  // Enregistrez le temps de fin

        // Calculer le temps d'exécution en millisecondes
        double executionTime = ((double)(end - start) / CLOCKS_PER_SEC) * 1000;

        printf("Execution Time: %f ms\n", executionTime);

        // Vérifier si le temps d'exécution est supérieur à 80 ms
        if (executionTime > EXECUTION_LIMIT)
        {
            const char *taskName = "vSearch";  // Changez le nom de la tâche ici
            printf("La tâche %s n'est pas ordonnancée. Temps d'exécution réel: %f ms au lieu de %f ms.\n", taskName, executionTime, EXECUTION_LIMIT);
            vTaskEndScheduler();  // Arrêter le scheduler FreeRTOS
        }
    }
}

/*-----------------------------------------------------------*/

static void prvQueueSendTimerCallback( TimerHandle_t xTimerHandle )
{
    const uint32_t ulValueToSend = mainVALUE_SENT_FROM_TIMER;

    /* This is the software timer callback function.  The software timer has a
     * period of two seconds and is reset each time a key is pressed.  This
     * callback function will execute if the timer expires, which will only happen
     * if a key is not pressed for two seconds. */

    /* Avoid compiler warnings resulting from the unused parameter. */
    ( void ) xTimerHandle;

    /* Send to the queue - causing the queue receive task to unblock and
     * write out a message.  This function is called from the timer/daemon task, so
     * must not block.  Hence the block time is set to 0. */
    xQueueSend( xQueue, &ulValueToSend, 0U );
}
/*-----------------------------------------------------------*/








/*-----------------------------------------------------------*/
