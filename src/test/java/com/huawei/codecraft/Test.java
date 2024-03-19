package com.huawei.codecraft;


import junit.framework.TestCase;

import java.io.*;
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

    public void test5() throws IOException {
        char[][] map = new char[200][200];
        File doc = new File("C:\\Users\\FCG\\Desktop\\map1.txt");
        int i=0;
        BufferedReader obj = new BufferedReader(new FileReader(doc));
        String strng;
        while ((strng = obj.readLine()) != null) {
            map[i++] = strng.toCharArray();
        }
//        char[][] map = {
//                {'*', '*', '*', '*', '*', '*', '*', '*', '*', '*',},
//                {'#', '.', '.', '.', '.', '.', '.', '.', '.', '#'},
//                {'#', '.', '.', '.', '.', '#', '.', '.', '.', '#'},
//                {'#', '.', '.', '.', '#', '.', '.', '.', '.', '#'},
//                {'#', '.', '.', '.', '#', '.', '.', '.', '.', '#'},
//                {'#', '.', '.', '.', '.', '#', '.', '.', '.', '#'},
//                {'#', '.', '.', '.', '#', '.', '.', '.', '.', '#'},
//                {'#', '.', '.', '.', '.', '#', '.', '.', '.', '#'},
//                {'#', '.', '.', '.', '.', '.', '.', '.', '.', '#'},
//                {'#', '*', '*', '*', '*', '*', '*', '*', '*', '#'}
//        };
        boolean[][] gds_map = new boolean[200][200];
        Bfs bfs = new Bfs(map,gds_map);
        Astar astar = new Astar(map);
        long l = System.currentTimeMillis();
//        Queue<int[]> q = astar.aStarSearchBerth(new int[]{103, 93}, new int[]{4, 195});
        Queue<int[]> q = bfs.bfsSearchGood(new int[]{103, 93}, new int[]{4, 195});
        long e = System.currentTimeMillis();
        q.forEach(ints -> System.out.println(Arrays.toString(ints)));
        System.out.println(e-l);
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
