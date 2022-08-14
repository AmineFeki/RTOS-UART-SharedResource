# RTOS-UART-SharedResource

Let’s consider this application:
Four tasks T1, T2, T3, T4. Each task sends via UART the message “Hello from task i”, where i is the task number.
T1: Low Priority, Periodicity: 400 ms
T2: Low Priority 1, Periodicity: 300 ms
T3: Low Priority 2, Periodicity: 200 ms
T4: Low Priority 3, Periodicity: 100 ms

Configure the FreeRTOS in the (.ioc) file. We obtain the following configuration:

```
/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
```

Then, we create the task: ```  Task1Handle = osThreadNew(Task1Function, NULL, &Task1_attributes);```

Then, we start the Kernel: ```  osKernelStart();```

Here is the definition of the task function: 
```
/* USER CODE END Header_Task1Function */
void Task1Function(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t occurence1 = 0;
  /* Infinite loop */
  for(;;)
  {
	  osDelay(PERIODICITY_TASK1);
	  HAL_UART_Transmit(&huart3, (uint8_t*)RTOS_TaskMsg1_ac, TX_BUFFER_SIZE, TX_MAX_DELAY);
  }
  /* USER CODE END 5 */
}
```
We run the code and we obtain the following result:
![image](https://user-images.githubusercontent.com/53936812/184529915-932d6a11-5225-4a82-8c48-6e26ae0b2595.png)

Interpretation:
•	According to this result we can conclude that the UART itself is not facing any problems: When it begins sending, it is never preempted, and it sends all the data. (Unexpected result).
•	Assuming that the data sent includes the number of times a task is executed (the number at the end), and considering the result in the red rectangle, there some messages that are not sent.
==> Protection in the function HAL_UART_Transmit.

To remove this protection, we just need to define another function called FEKI_UART_Transmit without any protection.
Remove the following block and just keep the mentioned block:
```
  /* Check that a Tx process is not already ongoing */
  if (huart->gState == HAL_UART_STATE_READY)
  {
      /* Keep the code here */
   }
  else
  {
    return HAL_BUSY;
  }
```

Let’s try it! 

```
/* USER CODE END Header_Task1Function */
void Task1Function(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t occurence1 = 0;
  /* Infinite loop */
  for(;;)
  {
	  osDelay(PERIODICITY_TASK1);
	  FEKI_UART_Transmit(&huart3, (uint8_t*)RTOS_TaskMsg1_ac, TX_BUFFER_SIZE, TX_MAX_DELAY);
  }
  /* USER CODE END 5 */
}
```

We obtain the following result!

![image](https://user-images.githubusercontent.com/53936812/184530064-33901fdc-aaf2-4bdf-b642-09a7e26c9627.png)


==>	Logical result!

==>	Need to implement protection using binary semaphore! 


## Use semaphore:

In our case, UART is a shared resource and the code that sends data is a critical section.
Due to the problems previously mentioned, we have to protect this critical section. 

==>	We use binary semaphore.

To do so, we configure and create a binary semaphore, then we use the APIs 

``` osSemaphoreAcquire(Semaphore1Handle, portMAX_DELAY) and osSemaphoreRelease(Semaphore1Handle);. ```

### Code:

```
if (osOK == osSemaphoreAcquire(Semaphore1Handle, portMAX_DELAY))
{
RTOS_TaskMsg4_ac[18] = (char)(occurence4 + 48);
	FEKI_UART_Transmit(&huart3, (uint8_t*)RTOS_TaskMsg4_ac, TX_BUFFER_SIZE, TX_MAX_DELAY);
osSemaphoreRelease(Semaphore1Handle);
}
else
{
/* do nothing */
}
```

We obtain the following result:

![image](https://user-images.githubusercontent.com/53936812/184530234-79b5164a-6278-4f06-9923-b5a4c836ff87.png)

### Interpretation:

•	After using binary semaphores, we obtain a good enough result that allows us to protect the critical section and have full messages. None of the messages is preempted.
