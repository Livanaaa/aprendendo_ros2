from setuptools import find_packages, setup

package_name = 'meu_primeiro_pacote'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
        data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/meu_primeiro_launch.py']),
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
            'meu_primeiro_no = meu_primeiro_pacote.no_simples:main',
            'no = meu_primeiro_pacote.no_com_classe:main',
            'talker = meu_primeiro_pacote.talker:main',
            'listener = meu_primeiro_pacote.listener:main',
            'r2d2 = meu_primeiro_pacote.r2d2:main',
            'r2d2_controle = meu_primeiro_pacote.r2d2_controle:main',
            'wall = meu_primeiro_pacote.wall:main',
            'wavefront = meu_primeiro_pacote.wavefront:main',
            'rrt = meu_primeiro_pacote.rrt:main',
            'wavefront_2 = meu_primeiro_pacote.wavefront_2:main',

        ],

    },
)
