#!/bin/bash

echo "0000:26:00.0" > "/sys/bus/pci/drivers/xhci_hcd/unbind"
echo "0000:26:00.0" > "/sys/bus/pci/drivers/xhci_hcd/bind"

sleep 2
