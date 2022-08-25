import os
import paramiko

class FileTransporterSFTPClient(paramiko.SFTPClient):
    """
    Subclass of SFTPClient to achieve directory transport
    """
    def put_dir(self, source, target):
        for item in os.listdir(source):
            if os.path.isfile(os.path.join(source, item)):
                self.put(os.path.join(source, item), '%s/%s' % (target, item))
            else:
                self.mkdir('%s/%s' % (target, item), ignore_existing=True)
                self.put_dir(os.path.join(source, item), '%s/%s' % (target, item))

    def mkdir(self, path, mode=511, ignore_existing=False):
        try:
            super(FileTransporterSFTPClient, self).mkdir(path, mode)
        except IOError:
            if ignore_existing:
                pass
            else:
                raise
    def rmall(self,path):
        try:
            files = self.listdir(path)
        except IOError:
            return
        for f in files:
            filepath = os.path.join(path, f)
            try:
                self.remove(filepath)
            except IOError:
                self.rmall(filepath)
        self.rmdir(path)




def create_clients(IP_ADRESS, USERNAME, PASSWORD):
    """
    Helper function that creates the SSH and SFTP clients
    
    Arguments:
        - IP_ADRESS(str): Remote computer adress
        - USERNAME(str): Remote computer login name
        - PASSWORD(str): Remote login password
    """
    SSH_client=paramiko.SSHClient()
    SSH_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    SSH_client.connect(IP_ADRESS, username=USERNAME, password=PASSWORD)

    # get transport & open 
    transport=SSH_client.get_transport()
    SFTP_client=FileTransporterSFTPClient.from_transport(transport)

    return SSH_client, SFTP_client

def create_environment(ROS_MASTER_URI, IP_ADRESS, path):
    """
    Helper function that creates a unique env.sh file for every machine during installation.
    The env.sh is later used to source the environment on remote launches.

    Arguments:
        - ROS_MASTER_URI: The IP adress & port of the ROS master
        - IP_ADRESS: The IP adress of the machine where the environment will be sourced
        - path: env.sh file path    
    """

    with open(path, 'w') as f:
        # source environment/workspace
        f.write('#! /usr/bin/env bash\nsource /opt/ros/melodic/setup.bash\nsource $( cd -- "$( dirname --"${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/aimotion-f1tenth-system/devel/setup.bash\n')
        # add environment variables
        f.write(f'export ROS_MASTER_URI="{ROS_MASTER_URI}"\nexport ROS_IP={IP_ADRESS}\nexec "$@"')
        f.close()
