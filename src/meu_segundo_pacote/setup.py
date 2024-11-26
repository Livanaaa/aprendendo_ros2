from setuptools import find_packages, setup

package_name = 'meu_segundo_pacote'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
        data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/meu_segundo_launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lívia Casarini',
    maintainer_email='unielcasarini@fei.edu.br',
    description='Exemplo de construção do meu primeiro pacote no Ros2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'meu_segundo_no = meu_segundo_pacote.meu_segundo_no:main',
        ],
        'console_scripts': [
            'meu_terceiro_no = meu_segundo_pacote.meu_terceiro_no:main',
        ],
        
    },
)
