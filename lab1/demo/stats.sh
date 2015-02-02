echo "seq 100 run\n" >> data.txt

for V in {1..3}
 do 
    ./demo --seq --timing-mode
done

echo "pthread 2 100 run \n" << data.txt

for E in {1..100}
 do 
    ./demo --pthread threads=2 --timing-mode
done

echo "pthread 4 100 run \n" << data.txt

for V in {1..100}
 do 
    ./demo --pthread threads=4 --timing-mode
done

echo "pthread 8 100 run \n" << data.txt
for V in {1..100}
 do 
    ./demo --pthread threads=8 --timing-mode
done

echo "pthread 16 100 run \n" << data.txt
for V in {1..100}
 do 
    ./demo --pthread threads=16 --timing-mode
done

echo "omp 2 100 run \n" << data.txt
for V in {1..100}
 do 
    ./demo --omp --threads=2 --timing-mode
done

echo "omp 4 100 run \n" << data.txt
for V in {1..100}
 do 
    ./demo --omp --threads=4 --timing-mode
done

echo "omp 8 100 run \n" << data.txt
for V in {1..100}
 do 
    ./demo --omp --threads=8 --timing-mode
done

echo "omp 16 100 run \n" << data.txt
for V in {1..100}
 do 
    ./demo --omp --threads=16 --timing-mode
done

