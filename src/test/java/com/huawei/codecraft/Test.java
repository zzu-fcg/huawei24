package com.huawei.codecraft;


import junit.framework.TestCase;
import sun.text.resources.cldr.ti.FormatData_ti;

import java.sql.SQLOutput;
import java.util.*;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadPoolExecutor;

public class Test extends TestCase {
    //public Main3 main = new Main3();

    public void test1() {
        char[][] map = {
                {'*', '*', '*', '*', '*', '*', '*', '*', '*', '*'},
                {'#', '.', '.', '.', '.', '.', '.', '.', '.', '#'},
                {'#', '.', '.', '.', '.', '#', '.', '.', '.', '#'},
                {'#', '.', '.', '.', '#', '.', '.', '.', '.', '#'},
                {'#', '.', '.', '.', '#', '.', '.', '.', '.', '#'},
                {'#', '.', '.', '.', '.', '#', '.', '.', '.', '#'},
                {'#', '.', '.', '.', '#', '.', '.', '.', '.', '#'},
                {'#', '.', '.', '.', '.', '#', '.', '.', '.', '#'},
                {'#', '.', '.', '.', '.', '.', '.', '.', '.', '#'},
                {'#', '*', '*', '*', '*', '*', '*', '*', '*', '#'}
        };
        Astar astar = new Astar(map);
        Queue<int[]> q = astar.aStarSearchBerth(new int[]{8, 1}, new int[]{1, 8});
        q.forEach(ints -> System.out.println(Arrays.toString(ints)));
        //main.getPath(map,new int[]{0,0}, new int[]{4,4}).forEach(ints -> System.out.println(Arrays.toString(ints)));
    }

    public void test2() {
        Map<Integer, Integer> berth_map = new HashMap<>();
        for (int i = 0; i < 10; i++) {
            berth_map.put(i, i);
        }
        List<Map.Entry<Integer, Integer>> list = new ArrayList<Map.Entry<Integer, Integer>>(berth_map.entrySet());
        list.sort((o1, o2) -> o2.getValue() - o1.getValue());
        list.forEach(p -> System.out.println(p.getKey() + " " + p.getValue()));
    }

    public void test3() {
        boolean[] semaphore = new boolean[10];
        Arrays.fill(semaphore, true);
        for (int j = 0; j < 150000; j++) {

            for (int i = 0; i < 10; i++) {
                if (!semaphore[i]) {
                    continue;
                }
                semaphore[i] = false;
                int idx = i;
                new Thread(() -> {
                    System.out.println(Thread.currentThread().getName());
                    semaphore[idx] = true;
                }).start();
            }
        }

    }

    public void test4() {
        ThreadPoolExecutor executor = (ThreadPoolExecutor) Executors.newFixedThreadPool(10);
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 10; j++) {
                long l = System.currentTimeMillis();
                executor.execute(() -> {
                    int id =10000;
                    while (id>0){
                        id--;
                    }
                });
                long e = System.currentTimeMillis();
                System.out.println(e-l);
            }
        }
    }
}
