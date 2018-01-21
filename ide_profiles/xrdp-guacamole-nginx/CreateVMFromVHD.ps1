# Variables for the Resource Group, Networking and the Virtual Machine
$ResourceGroupName  ="udacity"
$Location = "West Europe"
$VMName = "herbie-vm"

# Retreived from storage account in Azure Portal after VHD upload
$OSDiskUri = "https://daiudacity02.blob.core.windows.net/vhds/udacity-herbie-fixed.vhd"
$VMSize = "Standard_NC6S_V2"

# Networking
$NSGName = "herbie-nsg"
$SubnetName = "herbie-subnet"
$InterfaceName = "herbie-interface"
$VNetName = "herbie-network"
$VNetResourceGroupName = "udacity"
$OSDiskName = $VMName

# Login to Azure
Login-AzureRMAccount

# Networking
# Create Network Security Group, Subnet and Virtual Network
$NSG = New-AzureRmNetworkSecurityGroup -Name $NSGName -ResourceGroupName $ResourceGroupName -Location $Location
$Subnet = New-AzureRMVirtualNetworkSubnetConfig -Name $SubnetName -AddressPrefix 10.1.1.0/24 -NetworkSecurityGroup $NSG
$VNet = New-AzureRmVirtualNetwork -Name $VNetName -ResourceGroupName $VNetResourceGroupName -Location $Location -AddressPrefix 10.1.1.0/24 -Subnet $Subnet

# Create the Interface
$pip = New-AzureRmPublicIpAddress -Name "$VMName-IP" -ResourceGroupName $ResourceGroupName -Location $Location -AllocationMethod Dynamic
$Interface  = New-AzureRMNetworkInterface -Name $InterfaceName -ResourceGroupName $ResourceGroupName -Location $Location -SubnetId $vnet.Subnets[0].Id -PublicIpAddressId $pip.Id

# VM Details
$VirtualMachine  = New-AzureRMVMConfig -VMName $VMName -VMSize $VMSize # -AvailabilitySetID $AvailabilitySet.Id
$VirtualMachine  = Add-AzureRMVMNetworkInterface -VM $VirtualMachine -Id $Interface.Id
$VirtualMachine  = Set-AzureRMVMOSDisk -VM $VirtualMachine -Name $OSDiskName -VhdUri $OSDiskUri -CreateOption Attach -Windows

# Create the VM in Azure
New-AzureRMVM -ResourceGroupName $ResourceGroupName -Location $Location -VM $VirtualMachine
