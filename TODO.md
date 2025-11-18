# TODO: Apply Solutions to Fix Simulation Issues

## 1. Signal Initialization
- [ ] ula32.vhd: Initialize signals like s_temp, soma_temp, carry_temp, novo_B, i_temp, igual_temp, overflow_temp at declaration to avoid 'X'.
- [ ] Memoria.vhd: Ensure addr and other signals are handled properly.
- [ ] Banco_reg.vhd: Ensure Cluster array is initialized on reset.
- [ ] Registrador.vhd: Already initializes Saida on reset, verify.
- [ ] cpu.v: Ensure wires are assigned properly, add defaults if needed.
- [ ] controlUnit.v: Change reset to asynchronous, set reset_start to 0, initialize state to 0, ensure all regs are 0 on reset.

## 2. Handling Undefined Values in to_integer
- [ ] Memoria.vhd: Add check before to_integer on Address to avoid 'X' or 'U'.
- [ ] Banco_reg.vhd: Add checks for ReadReg1, ReadReg2, WriteReg before to_integer.

## 3. Library Dependencies
- [ ] Verify all files use NUMERIC_STD and compatible libraries. Already done in revisions.

## 4. Reset Correction
- [x] controlUnit.v: Make reset asynchronous to ensure all values are zero on initial reset and prevent PC from updating during reset.
- [x] controlUnit.v: Initialize all output regs to 0 to avoid 'X' values.

## Followup Steps
- [ ] Simulate the project after changes to check for warnings.
- [ ] Test compilation and synthesis.
