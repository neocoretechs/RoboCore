package com.neocoretechs.robocore;

import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.TimeUnit;

/**
 * Class to manage thread resources throughout the application. Singleton. Fixed thread pool.
 * Attached a run completion method that decrements a countdown latch until all threads up to executionLimit
 * have completed.
 * @author jg
 *
 */
public class SynchronizedFixedThreadPoolManager {
	int threadNum = 0;
    //DaemonThreadFactory dtf ;//= new PoolThreadFactory();
    //private static Map<String, ExecutorService> executor = new HashMap<String, ExecutorService>();// = Executors.newCachedThreadPool(dtf);
    private static Map<String, FactoryThreadsLimit> executor = new HashMap<String, FactoryThreadsLimit>();
	public static SynchronizedFixedThreadPoolManager threadPoolManager = null;
	private SynchronizedFixedThreadPoolManager() { }
	/**
	 * Create a manager with the number of working threads and total threads to execute in the
	 * default SYSTEMSYNC group
	 * @param maxExecution
	 * @param executionLimit
	 * @return
	 */
	public static SynchronizedFixedThreadPoolManager getInstance(int maxThreads, int executionLimit) {
		if( threadPoolManager == null ) {
			threadPoolManager = new SynchronizedFixedThreadPoolManager();
			//threadPoolManager.dtf = getInstance(maxExecution, executionLimit).new DaemonThreadFactory("SYSTEMSYNC");
			//threadPoolManager.totalThreads = executionLimit;
			DaemonThreadFactory dtf = (getInstance(maxThreads, executionLimit).new DaemonThreadFactory("SYSTEMSYNC"));
			ExecutorService tpx = (getInstance(maxThreads, executionLimit).new ExtendedExecutor(maxThreads, executionLimit, new ArrayBlockingQueue<Runnable>(executionLimit), dtf));
			executor.put("SYSTEMSYNC", 
					(getInstance(maxThreads, executionLimit).new FactoryThreadsLimit("SYSTEMSYNC", dtf, tpx, maxThreads, executionLimit)));
			((ExtendedExecutor)tpx).prestartAllCoreThreads();
			//executor.put("SYSTEMSYNC",tpx );		
		}
		return threadPoolManager;
	}
	/**
	 * Create a manager with the number of working threads and total threads to execute in the
	 * default named group
	 * @param maxExecution
	 * @param executionLimit
	 * @param group The group name, to differentiate between working sets of different operation or thread counts
	 * @return
	 */
	public static SynchronizedFixedThreadPoolManager getInstance(int maxThreads, int executionLimit, String group) {
		if( threadPoolManager == null ) {
			threadPoolManager = new SynchronizedFixedThreadPoolManager();
			DaemonThreadFactory dtf = (getInstance(maxThreads, executionLimit).new DaemonThreadFactory(group));
			ExecutorService tpx = (getInstance(maxThreads, executionLimit).new ExtendedExecutor(maxThreads, executionLimit, new ArrayBlockingQueue<Runnable>(executionLimit), dtf));
			executor.put(group, 
					(getInstance(maxThreads, executionLimit).new FactoryThreadsLimit(group, dtf, tpx, maxThreads, executionLimit)));
			((ExtendedExecutor)tpx).prestartAllCoreThreads();
		}
		return threadPoolManager;
	}
	
	public static SynchronizedFixedThreadPoolManager getInstance() {
		if( threadPoolManager == null )
			threadPoolManager = new SynchronizedFixedThreadPoolManager();
		return threadPoolManager;
	}
	
	public void init(int maxThreads, int executionLimit, String group) {
		FactoryThreadsLimit ftl = executor.get(group);
		if( ftl != null ) {
			ftl.exs.shutdownNow();
			executor.remove(ftl.group);
		}
		DaemonThreadFactory dtf = new DaemonThreadFactory(group);
		ExecutorService tpx = new ExtendedExecutor(maxThreads, executionLimit, new ArrayBlockingQueue<Runnable>(executionLimit), dtf);
		executor.put(group, new FactoryThreadsLimit(group, dtf, tpx, maxThreads, executionLimit));
		((ExtendedExecutor)tpx).prestartAllCoreThreads();
	}
	
	/**
	 * Create an array of Executors that manage a cached thread pool for
	 * reading topics. One thread pool per topic to notify listeners of data ready
	 * @param threadGroupNames The topics for which thread groups are established
	 */
	public static void init(int maxThreads, int executionLimit, String[] threadGroupNames) {
		for(String tgn : threadGroupNames) {
			//executor.put(tgn, getInstance(maxExecution, executionLimit).new ExtendedExecutor(maxExecution, executionLimit, new ArrayBlockingQueue<Runnable>(executionLimit)));
			DaemonThreadFactory dtf = (getInstance(maxThreads, executionLimit).new DaemonThreadFactory(tgn));
			ExecutorService tpx = (getInstance(maxThreads, executionLimit).new ExtendedExecutor(maxThreads, executionLimit, new ArrayBlockingQueue<Runnable>(executionLimit), dtf));
			executor.put(tgn,
					(getInstance(maxThreads, executionLimit).new FactoryThreadsLimit(tgn, dtf, tpx, maxThreads, executionLimit)));
			((ExtendedExecutor)tpx).prestartAllCoreThreads();
		}
	}
	
	public void waitForGroupToFinish(String group) throws InterruptedException {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group));
		ExecutorService exe = ftl.exs;
		((ExtendedExecutor)exe).getLatch().await();
		// now reset latch
		((ExtendedExecutor)exe).latch = new CountDownLatch(ftl.maxExecution);
	}
	
	public void waitForGroupToFinish() throws InterruptedException {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get("SYSTEMSYNC"));
		ExecutorService exe = ftl.exs;
		((ExtendedExecutor)exe).getLatch().await();
		// now reset latch
		((ExtendedExecutor)exe).latch = new CountDownLatch(ftl.maxExecution);
	}
	
	public BlockingQueue<Runnable> getQueue(String group) {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group));
		ExecutorService exe = ftl.exs;
		return ((ExtendedExecutor)exe).getQueue();
		//return ((ThreadPoolExecutor)executor.get(group)).getQueue();
	}
	
	public BlockingQueue<Runnable> getQueue() {
		//return ((ThreadPoolExecutor)executor.get("SYSTEMSYNC")).getQueue();
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get("SYSTEMSYNC"));
		ExecutorService exe = ftl.exs;
		return ((ExtendedExecutor)exe).getQueue();
	}

	
	public void waitGroup(String group) {
		try {
			FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group));
			ExecutorService w = ftl.exs;
			//ExecutorService w = executor.get(group);
			synchronized(w) {
				w.wait();
			}
		} catch (InterruptedException e) {
		}
	}
	
	public void waitGroup(String group, long millis) {
		try {
			FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group));
			ExecutorService w = ftl.exs;
			//ExecutorService w = executor.get(group);
			synchronized(w) {
				w.wait(millis);
			}
		} catch (InterruptedException e) {
		}
	}
	
	public void notifyGroup(String group) {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group));
		ExecutorService w = ftl.exs;
		//ExecutorService w = executor.get(group);
		synchronized(w) {
			w.notifyAll();
		}
	}
	
	public void spin(Runnable r, ThreadGroup group) {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group.getName()));
		ExecutorService exe = ftl.exs;
	    /*executor.get(group.getName())*/exe.execute(r);
	}
	
	public void spin(Runnable r, String group) {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get(group));
		ExecutorService exe = ftl.exs;
	    /*executor.get(group)*/exe.execute(r);
	}
	
	public void spin(Runnable r) {
		FactoryThreadsLimit ftl = ((FactoryThreadsLimit)executor.get("SYSTEMSYNC"));
		ExecutorService exe = ftl.exs;
	    /*executor.get("SYSTEMSYNC")*/exe.execute(r);
	}
	
	public void shutdown() {
		//Collection<ExecutorService> ex = executor.values();
		Collection<FactoryThreadsLimit> ex = executor.values();
		for(/*ExecutorService*/FactoryThreadsLimit e : ex) {
			List<Runnable> spun = e.exs.shutdownNow();
			for(Runnable rs : spun) {
				System.out.println("Marked for Termination:"+rs.toString()+" "+e.toString());
			}
		}
	}
	
	class DaemonThreadFactory implements ThreadFactory {
		ThreadGroup threadGroup;
	
		public DaemonThreadFactory(String threadGroupName) {
			threadGroup = new ThreadGroup(threadGroupName);
		}	
		public ThreadGroup getThreadGroup() { return threadGroup; }		
	    public Thread newThread(Runnable r) {
	        Thread thread = new Thread(threadGroup, r, threadGroup.getName()+(++threadNum));
	        //thread.setDaemon(true);
	        return thread;
	    }
	}
	
	class ExtendedExecutor extends ThreadPoolExecutor {
		public CountDownLatch latch;
		public ExtendedExecutor(int maxThreads, int executionLimit, BlockingQueue<Runnable> threadQueue, DaemonThreadFactory dtf) {
			super(maxThreads, executionLimit, Long.MAX_VALUE, TimeUnit.DAYS, threadQueue, dtf );
			latch = new CountDownLatch(executionLimit);
		}
		public CountDownLatch getLatch() { return latch; }
		
		@Override
		protected void afterExecute(Runnable r, Throwable t) {
			super.afterExecute(r, t);
		  // regardless of errors, etc, count down the latch
			latch.countDown();
		}
		
	}
	
	class FactoryThreadsLimit {
		public DaemonThreadFactory dtf;
		public ExecutorService exs;
		public int totalThreads;
		public int maxExecution;
		public String group;
		public FactoryThreadsLimit(String group, DaemonThreadFactory dtf, ExecutorService exs, int totalThreads, int maxExecution ) {
			this.group = group;
			this.dtf = dtf;
			this.exs = exs;
			this.totalThreads = totalThreads;
			this.maxExecution = maxExecution;
		}
		@Override
		public boolean equals(Object o) {
			return group.equals(((FactoryThreadsLimit)o).group);
		}
		@Override
		public int hashCode() {
			return group.hashCode();
		}
	}
}
