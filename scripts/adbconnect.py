from subprocess import run, Popen, PIPE, DEVNULL
from time import sleep
import sys, os


RC_PACKAGE = 'com.qualcomm.ftcrobotcontroller'
RC_ACTIVITY = 'org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity'
ADB_PORT = 5555
PROFILE_FILENAME = 'temp.xml'
WIFI_DIRECT_PREFIX = 'DIRECT-xx-'

PROFILE_XML = '''<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
    <name>{ssid}</name>
    <SSIDConfig>
        <SSID>
            <name>{ssid}</name>
        </SSID>
    </SSIDConfig>
    <connectionType>ESS</connectionType>
    <connectionMode>auto</connectionMode>
    <MSM>
        <security>
            <authEncryption>
                <authentication>WPA2PSK</authentication>
                <encryption>AES</encryption>
                <useOneX>false</useOneX>
            </authEncryption>
            <sharedKey>
                <keyType>passPhrase</keyType>
                <protected>false</protected>
                <keyMaterial>{passphrase}</keyMaterial>
            </sharedKey>
        </security>
    </MSM>
    <MacRandomization xmlns="http://www.microsoft.com/networking/WLAN/profile/v3">
        <enableRandomization>false</enableRandomization>
    </MacRandomization>
</WLANProfile>'''


def list_devices():
    output = run(['adb', 'devices', '-l'], stdout=PIPE).stdout.decode('ascii').split('\r\n')
    devices = []
    for line in output[1:]:
        if line == '':
            continue
        # TODO add more information about the device
        devices.append(line.split()[0])
    return devices


def list_wifi_networks():
    if os.name == 'nt':
        output = run(['netsh', 'wlan', 'show', 'networks'], stdout=PIPE).stdout.decode('ascii').split('\r\n')
        networks = []
        for line in output:
            if line.startswith('SSID'):
                networks.append(line.split()[-1])
        return networks
    else:
        print('Failure: wifi network listing not support on {}'.format(os.name))


def connect_to_wifi_network(network, passphrase):
    if os.name == 'nt':
        # create and add profile for the network
        print('Creating profile')
        with open(PROFILE_FILENAME, mode='w') as fh:
            fh.write(PROFILE_XML.format(ssid=network, passphrase=passphrase))
        print('Loading profile')
        run(['netsh', 'wlan', 'add', 'profile', 'filename="{}"'.format(PROFILE_FILENAME)], stdout=DEVNULL)
        os.remove('temp.xml')
        # update the network as necessary
        i = 0
        while True:
            output_lines = run(['netsh', 'wlan', 'show', 'interfaces'], stdout=PIPE).stdout.decode('ascii').split('\r\n')
            fields = {(line.split(':')[0].strip()): (line.split(':')[-1].strip()) for line in output_lines if ' :' in line}
            if fields['State'] == 'connected' and fields['SSID'] == network:
                i += 1
                if i >= 5:
                    print('Connected to {}'.format(fields['SSID']))
                    return
            elif (fields['State'] == 'disconnected') or (fields['State'] == 'connected' and fields['SSID'] != network):
                i = 0
                run(['netsh', 'wlan', 'connect', network], stdout=DEVNULL)
                print('Attempting to connect')
            else:
                i = 0
            sleep(0.25)
    else:
        print('Failure: wifi network listing not support on {}'.format(os.name))


if __name__ == '__main__':
    print('Connecting to device')
    while True:
        devices = list_devices()
        print('Devices:')
        for i, device in enumerate(devices):
            print('{}: {}'.format(i, device))
        device_input = input('Select device: ')
        if device_input != '':
            os.environ['ANDROID_SERIAL'] = devices[int(device_input)]
            break
    print('Restarting robot controller')
    if run(['adb', 'shell', 'am', 'force-stop', RC_PACKAGE], stdout=DEVNULL).returncode != 0:
        print('Failure: unable to restart robot controller')
        sys.exit(-1)
    if run(['adb', 'shell', 'am', 'start', '-n', '{}/{}'.format(RC_PACKAGE, RC_ACTIVITY)],
           stdout=DEVNULL).returncode != 0:
        print('Failure: unable to start robot controller')
        sys.exit(-1)
    print('Scanning logcat for passphrase')
    passphrase, wifi_name = None, None
    proc = Popen(['adb', 'logcat'], stdout=PIPE, universal_newlines=True, encoding='utf-8')
    for line in iter(proc.stdout.readline, ''):
        if 'passphrase' in line.lower():
            passphrase = line.split()[-1]
        if 'device information' in line.lower():
            wifi_name = line.split()[-2]
        if not (passphrase is None or wifi_name is None):
            break
    proc.kill()
    print('Got WiFi passphrase: {}'.format(passphrase))
    print('Got WiFi direct name: {}'.format(wifi_name))
    searching = True
    while searching:
        networks = list_wifi_networks()
        for network in networks:
            if network[len(WIFI_DIRECT_PREFIX):] == wifi_name:
                connect_to_wifi_network(network, passphrase)
                searching = False
    print('Connecting over wireless ADB')
    run(['adb', 'tcpip', str(ADB_PORT)], stdout=DEVNULL)
    run(['adb', 'connect', '192.168.49.1:{}'.format(ADB_PORT)], stdout=DEVNULL)
    print('You may disconnect the device now')
