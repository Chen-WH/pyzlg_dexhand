#!/usr/bin/env python3
"""GUI application for controlling DexHand using sliders"""

import sys
import os
import time

import logging
from typing import Dict, Optional
from dataclasses import dataclass

from pyzlg_dexhand import (
    LeftDexHand,
    RightDexHand,
    ControlMode,
    ZCANWrapper,
    MockZCANWrapper,
    JointCommand
)

def main():
    zcan = ZCANWrapper()
    zcan.open(device_index=0)
    hand = LeftDexHand(zcan)
    i = 0
    current_val = 30
    pos_val = 0
    hand.clear_errors(use_broadcast=False)
    hand.move_joints(
    th_rot=JointCommand(position=0, current=current_val),
    th_mcp=JointCommand(position=0, current=current_val),
    th_dip=JointCommand(position=0, current=current_val),
    
    ff_spr=JointCommand(position=0, current=current_val),
    ff_mcp=JointCommand(position=pos_val, current=current_val),
    ff_dip=JointCommand(position=pos_val, current=current_val),
    
    mf_mcp=JointCommand(position=pos_val, current=current_val),
    mf_dip=JointCommand(position=pos_val, current=current_val),
    
    rf_mcp=JointCommand(position=pos_val, current=current_val),
    rf_dip=JointCommand(position=pos_val, current=current_val),
    
    lf_mcp=JointCommand(position=pos_val, current=current_val),
    lf_dip=JointCommand(position=pos_val, current=current_val),
    control_mode=ControlMode.MIT_TORQUE)
    '''
    while True:
        current_val = int(20 + 0.5*i)
        if i > 160:
            break
        pos_val = 0
        hand.clear_errors(use_broadcast=False)
        hand.move_joints(
        th_rot=JointCommand(position=30, current=current_val),
        th_mcp=JointCommand(position=0, current=current_val),
        th_dip=JointCommand(position=0, current=current_val),
        
        ff_spr=JointCommand(position=0, current=current_val),
        ff_mcp=JointCommand(position=pos_val, current=current_val),
        ff_dip=JointCommand(position=pos_val, current=current_val),
        
        mf_mcp=JointCommand(position=pos_val, current=current_val),
        mf_dip=JointCommand(position=pos_val, current=current_val),
        
        rf_mcp=JointCommand(position=pos_val, current=current_val),
        rf_dip=JointCommand(position=pos_val, current=current_val),
        
        lf_mcp=JointCommand(position=pos_val, current=current_val),
        lf_dip=JointCommand(position=pos_val, current=current_val),
        control_mode=ControlMode.MIT_TORQUE)
        time.sleep(0.05)'''

if __name__ == "__main__":
    main()
