#!/usr/bin/zsh
echo "Plotting graph..."
gnuplot -persist << EOFMarker
    plot 'pid_controller.dat' using 1:2 w lines title 'Simulation Time', \
    'pid_controller.dat' using 1:3 w lines title 'Model Output', \
    'pid_controller.dat' using 1:4 w lines title 'Controller State'
EOFMarker