## Patient Queue
Patients are stored in data memory in a Queue (LIFO) data structure. They can be accessed by two functions:

### Enqueue:
Once a patient has entered their name into the kiosk and pressed D to confirm, call enqueue to assign a patient ID and add to queue. enqueue returns a pointer to the patient struct in data memory via r25:r24. This can be used to display the PID and name on the LCD screen for confirmation.

NOTE: Currently assumes keypad input is saved in a zero-padded, 8-byte variable in data memory called temp_name. Currently the keypad input is stored in a null-terminated, 9-byte variable - these need to be aligned.

### Dequeue:
Once the next patient arrives at the waiting room and the doctor presses the next patient, call dequeue to expose the next patient in the queue. This function does not return anything.

### Other notes:
#### Patient struct
Patients are stored on the queue in a struct with the following fields:
patient_id: 2 bytes
patient_name: 8 bytes

When a pointer to a patient is returned, it is a pointer to the lower order byte of patient_id.

#### Access patient at front of queue
To access the patient at the front of the queue, first check that variable num_patients != 0. If num_patients == 0, the queue is empty. If not empty, the address of the first patient of the queue can be found at data memory address first_patient. Use this pointer to read the patient name and PID only - please do not modify the patient struct directly.
