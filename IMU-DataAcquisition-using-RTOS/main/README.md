code structure to be written in main app:

// 1. Blink LED 3 times on boot
// 2. Initialize NVS
// 3. Check if bias exists in NVS
//    a. If exists, load and print
//    b. If not, store default bias values you gave above
// 4. Use the loaded bias for calibration logic
// 5. If bias successfully applied → blink 3 times (success)
//    If failure → blink 5 times (failure)
// 6. Print all biases using printf for debugging